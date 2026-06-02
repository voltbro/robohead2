from __future__ import annotations

import asyncio
import contextlib
import os
import signal
import ssl
import threading
from pathlib import Path
from typing import Any, AsyncIterator

import rclpy
import uvicorn
from fastapi import FastAPI, File, Form, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from rclpy.executors import MultiThreadedExecutor

from robohead_interfaces.msg import Color as ColorMsg, ColorArray
from robohead_interfaces.srv import Color as ColorSrv
from robohead_interfaces.srv import ColorPalette, Move, PlayMedia, SimpleCommand, Speak

from .image_utils import clamp, parse_color
from .ros_bridge import RoboheadBridge


class ConnectionManager:
    """Tracks browser WebSocket clients and broadcasts canvas commands."""

    def __init__(self) -> None:
        """Initialize the FastAPI app and mount static files for the web interface."""
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        """Accept and register a new websocket client."""
        await websocket.accept()
        async with self._lock:
            self._clients.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        """Unregister a websocket client when it disconnects."""
        async with self._lock:
            self._clients.discard(websocket)

    async def broadcast(self, message: dict[str, Any], *, exclude: WebSocket | None = None) -> None:
        """Send a JSON message to all connected websocket clients."""
        async with self._lock:
            clients = list(self._clients)
        dead: list[WebSocket] = []
        for client in clients:
            if client is exclude:
                continue
            try:
                await client.send_json(message)
            except Exception:
                dead.append(client)
        if dead:
            async with self._lock:
                for client in dead:
                    self._clients.discard(client)


class RoboheadWebServer:
    """FastAPI application bound to a RoboheadBridge ROS node."""

    def __init__(self, bridge: RoboheadBridge) -> None:
        """Initialize the FastAPI app and mount static files for the web interface."""
        self.bridge = bridge
        self.manager = ConnectionManager()
        self.app = FastAPI(title='Robohead Web', version='0.2.0')
        self.static_dir = self._resolve_static_dir()
        self.app.mount('/static', StaticFiles(directory=self.static_dir), name='static')
        self._install_routes()

    def _resolve_static_dir(self) -> Path:
        """Find the correct static assets directory, either locally or from the ROS install prefix."""
        here = Path(__file__).resolve().parent / 'static'
        if here.exists():
            return here
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
            candidate = Path(prefix) / 'share' / 'robohead_web' / 'static'
            if candidate.exists():
                return candidate
        return here

    def _install_routes(self) -> None:
        """Register all HTTP and websocket routes used by the Robohead web UI."""
        app = self.app

        @app.get('/')
        async def index() -> FileResponse:
            """Serve the main web page for the Robohead UI."""
            return FileResponse(self.static_dir / 'index.html')

        @app.get('/{script_name}.js')
        async def script_js(script_name: str) -> FileResponse:
            """Serve a requested JavaScript module by name from static files."""
            allowed_scripts = {
                'utils', 'ws', 'draw-panel', 'neck-panel', 'ears-panel',
                'tts-panel', 'led-panel', 'media-panel', 'audio-panel',
                'camera-panel', 'voice-panel', 'app',
            }
            if script_name not in allowed_scripts:
                raise HTTPException(status_code=404, detail='Not found')
            return FileResponse(
                self.static_dir / f'{script_name}.js',
                media_type='application/javascript',
            )

        @app.get('/app.js')
        async def app_js() -> FileResponse:
            """Serve the main application JavaScript bundle."""
            return FileResponse(self.static_dir / 'app.js', media_type='application/javascript')

        @app.get('/styles.css')
        async def styles_css() -> FileResponse:
            """Serve the web UI stylesheet."""
            return FileResponse(self.static_dir / 'styles.css', media_type='text/css')

        @app.get('/api/status')
        async def status() -> dict[str, Any]:
            """Return a compact robot status snapshot as JSON."""
            return self.bridge.status()

        @app.get('/api/canvas/history')
        async def canvas_history() -> dict[str, Any]:
            """Return the history of canvas drawing commands."""
            return {'commands': self.bridge.canvas_history_snapshot()}

        @app.get('/camera.mjpg')
        async def camera_mjpg() -> StreamingResponse:
            """Stream the robot camera as an MJPEG response."""
            return StreamingResponse(
                self._camera_stream(),
                media_type='multipart/x-mixed-replace; boundary=frame',
                headers={'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0', 'Pragma': 'no-cache'},
            )

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: WebSocket) -> None:
            """Accept a websocket connection for canvas and control messages."""
            await self._control_socket(websocket)

        @app.websocket('/audio_ws')
        async def audio_output(websocket: WebSocket) -> None:
            """Send live audio chunks to the browser over a websocket."""
            await self._audio_output_socket(websocket)

        @app.websocket('/audio_in_ws')
        async def audio_input(websocket: WebSocket) -> None:
            """Receive microphone audio from the browser and forward it to the robot."""
            await self._audio_input_socket(websocket)

        @app.post('/api/neck')
        async def neck(data: dict[str, Any]) -> dict[str, Any]:
            """Accept neck position commands from the web interface."""
            return {'ok': self.bridge.send_neck(data.get('vertical', 0), data.get('horizontal', 0), data.get('duration', 0.06))}

        @app.post('/api/ears')
        async def ears(data: dict[str, Any]) -> JSONResponse:
            """Accept ear angle commands and call the ROS service."""
            req = Move.Request()
            req.angle_a = clamp(data.get('left', 0), -90, 90)
            req.angle_b = clamp(data.get('right', 0), -90, 90)
            req.duration = float(data.get('duration', 0.25))
            return await self._call(self.bridge.ears_client, req)

        @app.post('/api/tts')
        async def tts(data: dict[str, Any]) -> JSONResponse:
            """Accept text-to-speech requests and call the voice service."""
            voices = {'aidar': 0, 'baya': 1, 'kseniya': 2, 'eugene': 3, 'xenia': 4}
            req = Speak.Request()
            req.text = str(data.get('text', ''))
            voice = data.get('voice', 'eugene')
            req.voice = int(voices.get(str(voice), voice if isinstance(voice, int) else 3))
            req.path_to_save = str(data.get('path_to_save', ''))
            req.put_accent = bool(data.get('put_accent', True))
            req.put_yo = bool(data.get('put_yo', True))
            req.play = bool(data.get('play', True))
            return await self._call(self.bridge.speak_client, req, timeout=20.0)

        @app.post('/api/volume')
        async def volume(data: dict[str, Any]) -> JSONResponse:
            """Accept volume requests and call the volume control service."""
            req = SimpleCommand.Request()
            req.data = clamp(data.get('value', 60), 0, 100)
            return await self._call(self.bridge.set_volume_client, req)

        @app.post('/api/media')
        async def media(data: dict[str, Any]) -> JSONResponse:
            """Accept media playback commands and call the media service."""
            req = PlayMedia.Request()
            req.path_to_video_file = str(data.get('video_path', ''))
            req.path_to_audio_file = str(data.get('audio_path', ''))
            req.loop = bool(data.get('loop', False))
            return await self._call(self.bridge.play_media_client, req)

        @app.post('/api/media/upload')
        async def media_upload(file: UploadFile = File(...), loop: bool = Form(False)) -> dict[str, Any]:
            """Handle uploaded media files and start playback on the robot."""
            return await self._media_upload(file, loop)

        @app.post('/api/led/mode')
        async def led_mode(data: dict[str, Any]) -> JSONResponse:
            """Set the LED mode via the robot service."""
            req = SimpleCommand.Request()
            req.data = int(data.get('mode', 0))
            return await self._call(self.bridge.led_mode_client, req)

        @app.post('/api/led/brightness')
        async def led_brightness(data: dict[str, Any]) -> JSONResponse:
            """Set the LED brightness via the robot service."""
            req = SimpleCommand.Request()
            value = clamp(data.get('value', 80), 0, 100)
            req.data = int(255 * value / 100)
            return await self._call(self.bridge.led_brightness_client, req)

        @app.post('/api/led/color')
        async def led_color(data: dict[str, Any]) -> JSONResponse:
            """Set the robot LED color via the color service."""
            r, g, b = parse_color(data.get('color', '#24d18b'))
            req = ColorSrv.Request()
            req.red, req.green, req.blue = r, g, b
            return await self._call(self.bridge.led_color_client, req)

        @app.post('/api/led/palette')
        async def led_palette(data: dict[str, Any]) -> JSONResponse:
            """Set the robot LED palette colors via the palette service."""
            a = parse_color(data.get('color_a', '#24d18b'))
            b = parse_color(data.get('color_b', '#ff4d6d'))
            req = ColorPalette.Request()
            req.color_a.red, req.color_a.green, req.color_a.blue = a
            req.color_b.red, req.color_b.green, req.color_b.blue = b
            return await self._call(self.bridge.led_palette_client, req)

        @app.post('/api/led/manual')
        async def led_manual(data: dict[str, Any]) -> dict[str, bool]:
            """Send a manual LED color array to the robot."""
            msg = ColorArray()
            msg.colors = []
            for raw in data.get('colors', [])[:12]:
                r, g, b = parse_color(raw)
                color = ColorMsg()
                color.red, color.green, color.blue = r, g, b
                msg.colors.append(color)
            while len(msg.colors) < 12:
                msg.colors.append(ColorMsg())
            self.bridge.pub_led_manual.publish(msg)
            return {'ok': True}

        @app.post('/api/draw/clear')
        async def draw_clear(data: dict[str, Any]) -> dict[str, bool]:
            """Clear the robot canvas and broadcast the clear command to clients."""
            cmd = self.bridge.clear_canvas(data.get('color', '#ffffff'), flush=True)
            await self.manager.broadcast(cmd)
            return {'ok': True}

    async def _call(self, client, request_msg, timeout: float = 4.0) -> JSONResponse:
        """Call a ROS service without blocking uvicorn's event loop."""
        if not client.service_is_ready():
            await asyncio.to_thread(client.wait_for_service, timeout_sec=0.2)
        if not client.service_is_ready():
            return JSONResponse({'ok': False, 'error': 'service_unavailable'}, status_code=503)

        future = client.call_async(request_msg)
        start = asyncio.get_running_loop().time()
        while not future.done():
            if asyncio.get_running_loop().time() - start > timeout:
                return JSONResponse({'ok': False, 'error': 'timeout'}, status_code=504)
            await asyncio.sleep(0.01)
        result = future.result()
        return JSONResponse({'ok': True, 'data': None if result is None else int(getattr(result, 'data', 0))})

    async def _camera_stream(self) -> AsyncIterator[bytes]:
        """Yield MJPEG chunks and exit quietly when a browser disconnects."""
        last_seq = -1
        while True:
            seq, jpeg = self.bridge.camera_frame(last_seq)
            if jpeg is not None:
                last_seq = seq
                yield b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n'
            await asyncio.sleep(0.025)

    async def _control_socket(self, websocket: WebSocket) -> None:
        """Handle control websocket events including drawing and client video."""
        await self.manager.connect(websocket)
        status_task = asyncio.create_task(self._status_loop(websocket))
        try:
            await websocket.send_json({'type': 'canvas_history', 'commands': self.bridge.canvas_history_snapshot()})
            while True:
                message = await websocket.receive()
                if message.get('type') == 'websocket.disconnect':
                    break
                if message.get('bytes'):
                    self.bridge.publish_client_video_frame(message['bytes'])
                    continue
                text = message.get('text')
                if text is None:
                    continue
                data = websocket_json(text)
                await self._handle_ws_command(websocket, data)
        except WebSocketDisconnect:
            pass
        finally:
            status_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await status_task
            await self.manager.disconnect(websocket)

    async def _handle_ws_command(self, websocket: WebSocket, data: dict[str, Any]) -> None:
        """Process incoming websocket commands from the browser."""
        kind = data.get('type')
        if kind == 'draw':
            cmd = self.bridge.draw_line(data, flush=bool(data.get('flush', False)))
            await self.manager.broadcast(cmd, exclude=websocket)
        elif kind == 'clear':
            cmd = self.bridge.clear_canvas(data.get('color', '#ffffff'), flush=True)
            await self.manager.broadcast(cmd, exclude=websocket)
        elif kind == 'neck':
            self.bridge.send_neck(data.get('vertical', 0), data.get('horizontal', 0), data.get('duration', 0.06))
        elif kind == 'client_audio':
            # Kept for old clients; new clients use /audio_in_ws binary frames.
            pass

    async def _status_loop(self, websocket: WebSocket) -> None:
        """Send periodic robot status updates over websocket."""
        while True:
            await websocket.send_json({'type': 'status', 'data': self.bridge.status()})
            await asyncio.sleep(0.5)

    async def _audio_output_socket(self, websocket: WebSocket) -> None:
        """Stream audio output frames to a websocket client."""
        await websocket.accept()
        await websocket.send_json({'type': 'config', 'sampleRate': 16000})
        last_seq = -1
        try:
            while True:
                seq, chunk = self.bridge.audio_chunk(last_seq)
                if chunk:
                    last_seq = seq
                    await websocket.send_bytes(chunk)
                await asyncio.sleep(0.006)
        except WebSocketDisconnect:
            pass

    async def _audio_input_socket(self, websocket: WebSocket) -> None:
        """Receive audio input packets from the browser and forward them to ROS."""
        await websocket.accept()
        try:
            while True:
                packet = await websocket.receive_bytes()
                self.bridge.publish_client_audio(packet)
        except WebSocketDisconnect:
            pass

    async def _media_upload(self, file: UploadFile, loop: bool) -> dict[str, Any]:
        """Save uploaded media to disk and request playback through ROS."""
        content = await file.read()
        if not content:
            raise HTTPException(status_code=400, detail='empty_file')
        if len(content) > 100 * 1024 * 1024:
            raise HTTPException(status_code=413, detail='file_too_large')
        ext = Path(file.filename or '').suffix.lower() or '.bin'
        upload_dir = Path('/tmp/robohead_media')
        upload_dir.mkdir(parents=True, exist_ok=True)
        path = upload_dir / f'web_upload{ext}'
        path.write_bytes(content)

        req = PlayMedia.Request()
        req.path_to_video_file = str(path)
        req.path_to_audio_file = str(path)
        req.loop = bool(loop)
        result = await self._call(self.bridge.play_media_client, req, timeout=10.0)
        if result.status_code >= 400:
            return {'ok': False, 'path': str(path), 'size': len(content)}
        return {'ok': True, 'path': str(path), 'size': len(content), 'loop': loop}


def websocket_json(text: str) -> dict[str, Any]:
    """Small JSON helper that keeps malformed WebSocket messages harmless."""
    import json

    try:
        value = json.loads(text)
    except Exception:
        return {}
    return value if isinstance(value, dict) else {}


def create_app(bridge: RoboheadBridge) -> FastAPI:
    """Create the FastAPI application using the provided ROS bridge."""
    return RoboheadWebServer(bridge).app


def main() -> None:
    """Initialize ROS, start the web server, and run the application."""
    rclpy.init()
    bridge = RoboheadBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(bridge)
    ssl_certfile = Path(__file__).resolve().parent / 'ssl' / 'cert.pem'
    ssl_keyfile =  Path(__file__).resolve().parent / 'ssl' / 'key.pem'

    config = uvicorn.Config(
        app,
        host=bridge.host,
        port=bridge.port,
        log_level='info',
        access_log=False,
        ssl_certfile=str(ssl_certfile) if ssl_certfile.exists() else None,
        ssl_keyfile=str(ssl_keyfile) if ssl_keyfile.exists() else None,
    )

    # config = uvicorn.Config(app, host=bridge.host, port=bridge.port, log_level='info', access_log=False)
    server = uvicorn.Server(config)

    try:
        server.run()
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
