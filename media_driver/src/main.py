#!/usr/bin/env python3
"""
ROS2 MediaDriver node using python-mpv (libmpv).
Provides services:
 - /media_driver/play_media  (robohead_interfaces/srv/PlayMedia)
 - /media_driver/set_volume  (robohead_interfaces/srv/SimpleCommand)
 - /media_driver/get_volume  (robohead_interfaces/srv/SimpleCommand)

Features:
 - images (png/jpg), video, audio
 - override audio (audio-add)
 - blocking playback (is_block)
 - loop (is_cycle)
 - simple fade-in for images to reduce flicker
"""

import rclpy
from rclpy.node import Node
import mpv
import os
import time
import threading
import urllib.request
from urllib.error import URLError, HTTPError
from robohead_interfaces.srv import PlayMedia, SimpleCommand

def is_url_accessible(url: str, timeout: float = 5.0) -> bool:
    try:
        req = urllib.request.Request(url, method='HEAD')
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            code = resp.getcode()
            return (200 <= code < 300) or (code == 206)
    except (HTTPError, URLError, ValueError):
        # Some servers don't allow HEAD — try GET but only read a few bytes
        try:
            with urllib.request.urlopen(url, timeout=timeout) as resp:
                # if we get even headers, consider accessible
                return True
        except Exception:
            return False
    except Exception:
        return False

def has_image_ext(path: str) -> bool:
    p = path.lower()
    return p.endswith('.png') or p.endswith('.jpg') or p.endswith('.jpeg') or p.endswith('.bmp') or p.endswith('.webp')

class MediaDriverNode(Node):
    def __init__(self):
        super().__init__('media_driver')

        # параметры (можно изменить через ROS2 params)
        self.declare_parameter('image_fade_ms', 80)  # fade duration in ms for images
        self.declare_parameter('hwdec', 'auto')
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('image_display_duration', 'inf')  # keep images
        self.declare_parameter('player_args', [])  # additional args if needed

        fade_ms = self.get_parameter('image_fade_ms').value
        hwdec = self.get_parameter('hwdec').value
        fullscreen = self.get_parameter('fullscreen').value
        image_display_duration = self.get_parameter('image_display_duration').value
        player_args = self.get_parameter('player_args').value

        # Инициализация mpv player (embedded libmpv)
        # важно: keep_open=True и image_display_duration=inf для стабильного отображения картинок
        mpv_options = {
            'hwdec': hwdec,
            'keep_open': 'yes',
            'image_display_duration': image_display_duration,
        }
        # build mpv constructor args
        constructor_kwargs = {
            'input_default_bindings': True,
            'input_vo_keyboard': False,
            'osc': False,
        }
        # pass options to mpv constructor
        try:
            self.player = mpv.MPV(**constructor_kwargs, **mpv_options)
        except TypeError:
            # older python-mpv versions might not accept options that way
            # fallback: create with defaults and set properties later
            self.player = mpv.MPV(**constructor_kwargs)
            for k, v in mpv_options.items():
                try:
                    setattr(self.player, k, v)
                except Exception:
                    pass

        # fullscreen if requested
        try:
            self.player.fullscreen = bool(fullscreen)
        except Exception:
            pass

        # internal state
        self.image_fade = float(fade_ms) / 1000.0  # seconds
        self._lock = threading.Lock()

        # services
        self.srv_play = self.create_service(PlayMedia, '/robohead_controller/media_driver/play_media', self.cb_play_media)
        self.srv_set_vol = self.create_service(SimpleCommand, '/robohead_controller/media_driver/set_volume', self.cb_set_volume)
        self.srv_get_vol = self.create_service(SimpleCommand, '/robohead_controller/media_driver/get_volume', self.cb_get_volume)

        self.get_logger().info('MediaDriverNode initialized (python-mpv).')

    def _apply_fade(self):
        """
        Adds a small fade-in filter to reduce flicker on images.
        Uses mpv vf add lavfi=[fade=in:st=0:d=X]
        """
        if self.image_fade <= 0:
            return
        # use VF (lavfi fade); if fails, ignore silently
        try:
            # add lavfi fade
            self.player.command('vf', 'add', f'lavfi=[fade=in:st=0:d={self.image_fade}]')
            # small delay to let mpv register filter
            time.sleep(0.01)
        except Exception as e:
            self.get_logger().debug(f'Failed to apply fade vf: {e}')

    def _clear_vf(self):
        # remove all filters (best-effort)
        try:
            self.player.command('vf', 'clear')
        except Exception:
            pass

    def cb_play_media(self, request: PlayMedia.Request, response: PlayMedia.Response) -> PlayMedia.Response:
        """
        Handle play_media service.
        Request fields: path_to_media_file, path_to_override_audio_file, is_block, is_cycle
        """
        response.data = -1
        path = request.path_to_media_file or ''
        override_audio = request.path_to_override_audio_file or ''
        is_block = bool(request.is_block)
        is_cycle = bool(request.is_cycle)

        if request.path_to_media_file:
            # self.player.stop()
            # self.player.play(request.path_to_media_file)
            self.player.command('loadfile', request.path_to_media_file, 'replace', 'reload')

            if request.path_to_override_audio_file:
                # disable embedded audio first
                try:
                    # aid no -> disable internal audio track
                    self.player.command('set_property', 'aid', 'no')
                except Exception:
                    pass
                # add external audio
                try:
                    self.player.command('audio-add', request.path_to_override_audio_file)
                except Exception:
                    self.get_logger().warn('audio-add failed (maybe unsupported)')
                # restore aid auto
                try:
                    self.player.command('set_property', 'aid', 'auto')
                except Exception:
                    pass
        else:
            self.player.stop()

        response.data = 0
        return response

        # special: empty path -> stop playback
        if path.strip() == '':
            try:
                self.player.stop()
            except Exception as e:
                self.get_logger().warn(f'stop() failed: {e}')
            response.data = 0
            return response

        # check URL or local file
        is_url = path.startswith('http://') or path.startswith('https://') or path.startswith('rtmp://')
        if is_url:
            if not is_url_accessible(path, timeout=5.0):
                self.get_logger().error(f'URL not accessible: {path}')
                response.data = -2
                return response
        else:
            if not os.path.exists(path):
                self.get_logger().error(f'Local file not found: {path}')
                response.data = -1
                return response

        # prepare override audio if given
        if override_audio:
            is_url2 = override_audio.startswith('http://') or override_audio.startswith('https://')
            if is_url2:
                if not is_url_accessible(override_audio, timeout=5.0):
                    self.get_logger().error(f'Override audio URL not accessible: {override_audio}')
                    response.data = -2
                    return response
            else:
                if not os.path.exists(override_audio):
                    self.get_logger().error(f'Override audio file not found: {override_audio}')
                    response.data = -1
                    return response

        # play logic (thread-safe)
        with self._lock:
            try:
                # If it's an image, apply fade to reduce visible flicker
                if has_image_ext(path):
                    # Use loadfile replace — libmpv embedded tends to be smoother than external mpv IPC.
                    # Apply small fade (best-effort)
                    self._apply_fade()
                    # load image
                    self.player.command('loadfile', path, 'replace')
                    # ensure image stays (image_display_duration set to inf)
                    # clear filters shortly after to avoid buildup
                    if self.image_fade > 0:
                        # schedule clearing of filter after short delay in background
                        threading.Thread(target=self._delayed_clear_vf, args=(self.image_fade + 0.05,), daemon=True).start()
                else:
                    # video/audio
                    # if override audio provided: we will set aid/no and audio-add AFTER loadfile
                    self.player.command('loadfile', path, 'replace')

                    if override_audio:
                        # disable embedded audio first
                        try:
                            # aid no -> disable internal audio track
                            self.player.command('set_property', 'aid', 'no')
                        except Exception:
                            pass
                        # add external audio
                        try:
                            self.player.command('audio-add', override_audio)
                        except Exception:
                            self.get_logger().warn('audio-add failed (maybe unsupported)')
                        # restore aid auto
                        try:
                            self.player.command('set_property', 'aid', 'auto')
                        except Exception:
                            pass

                # loop handling
                try:
                    if is_cycle:
                        # 'inf' for infinite loop
                        self.player.command('set_property', 'loop-file', 'inf')
                    else:
                        self.player.command('set_property', 'loop-file', 'no')
                except Exception:
                    pass

                # unpause
                try:
                    self.player.pause = False
                except Exception:
                    pass

                response.data = 0
                self.get_logger().info(f'Queued media: {path} (block={is_block} cycle={is_cycle})')
            except Exception as e:
                self.get_logger().error(f'Failed to queue media: {e}')
                response.data = -3
                return response

        # blocking behavior
        if is_block:
            # Wait until playback finishes. Use mpv event loop helper.
            # python-mpv provides wait_for_playback() which waits until EOF or stop.
            try:
                # wait_for_playback blocks until eof or stop. Provide a timeout safeguard.
                # We'll loop and check `eof-reached` property to allow node interruption.
                start_t = time.time()
                timeout_sec = 3600  # generous upper bound
                while rclpy.ok():
                    # use get_property for eof-reached
                    try:
                        eof = self.player.get_property('eof-reached')
                        if eof:
                            break
                    except Exception:
                        # if property not available, fallback to mpv.wait_for_playback if present
                        try:
                            # small blocking wait for mpv's playback to end
                            if hasattr(self.player, 'wait_for_playback'):
                                self.player.wait_for_playback()
                                break
                        except Exception:
                            pass
                    # minor sleep
                    time.sleep(0.05)
                    if time.time() - start_t > timeout_sec:
                        self.get_logger().warn('Blocking wait timed out')
                        break
            except Exception as e:
                self.get_logger().warn(f'Blocking wait failed: {e}')

        return response

    def _delayed_clear_vf(self, delay: float):
        """Clear vf after delay to avoid filter accumulation."""
        time.sleep(delay)
        with self._lock:
            try:
                self._clear_vf()
            except Exception:
                pass

    def cb_set_volume(self, request: SimpleCommand.Request, response: SimpleCommand.Response) -> SimpleCommand.Response:
        # set volume (0-100)
        vol = max(0, min(100, int(request.data)))
        try:
            self.player.volume = vol
            response.data = vol
        except Exception as e:
            self.get_logger().error(f'set volume failed: {e}')
            response.data = -1
        return response

    def cb_get_volume(self, request: SimpleCommand.Request, response: SimpleCommand.Response) -> SimpleCommand.Response:
        (void := request)  # keep signature
        try:
            vol = self.player.volume
            if vol is None:
                response.data = -1
            else:
                response.data = int(max(0, min(100, int(vol))))
        except Exception:
            response.data = -1
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MediaDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
