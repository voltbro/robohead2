import struct
import rclpy
from rclpy.node import Node

import pyaudio
import numpy as np
import usb.core
import usb.util
import math
import time
from std_msgs.msg import Bool, Int16, ColorRGBA
from robohead_msgs.msg import AudioData

# from respeaker_driver_dependencies.pixel_ring import PixelRing
# from respeaker_driver_dependencies.utils import *
# from respeaker_driver.msg import SetColorManualLED
# from respeaker_driver.srv import SetBrightnessLED, SetBrightnessLEDRequest, SetBrightnessLEDResponse
# from respeaker_driver.srv import SetColorAllLED, SetColorAllLEDRequest, SetColorAllLEDResponse
# from respeaker_driver.srv import SetColorPaletteLED, SetColorPaletteLEDRequest, SetColorPaletteLEDResponse
# from respeaker_driver.srv import SetModeLED, SetModeLEDRequest, SetModeLEDResponse

from contextlib import contextmanager
import os
import sys

@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield

class RespeakerDriver(Node):        
    def __init__(self):
        super().__init__('respeaker_driver')

        self.declare_parameter('usb/vendor_id', 0x2886)
        vendor_id = self.get_parameter('usb/vendor_id').get_parameter_value().integer_value

        self.declare_parameter('usb/product_id', 0x0018)
        product_id = self.get_parameter('usb/product_id').get_parameter_value().integer_value

        self.declare_parameter('usb/reset_timeout', 10)
        reset_timeout = self.get_parameter('usb/reset_timeout').get_parameter_value().integer_value

        self.declare_parameter('usb/io_timeout', 8000)
        self._io_timeout = self.get_parameter('usb/io_timeout').get_parameter_value().integer_value

        self.declare_parameter('audio/rate', 16000)
        rate = self.get_parameter('audio/rate').get_parameter_value().integer_value

        self.declare_parameter('audio/chunk', 1024)
        chunk = self.get_parameter('audio/chunk').get_parameter_value().integer_value


        self.declare_parameter('ros/topic_audio_main_name', "~/audio/main")
        topic_audio_main_name = self.get_parameter('ros/topic_audio_main_name').get_parameter_value().string_value

        self.declare_parameter('ros/topic_audio_channel_0_name', "~/audio/channel_0")
        self.declare_parameter('ros/topic_audio_channel_1_name', "~/audio/channel_1")
        self.declare_parameter('ros/topic_audio_channel_2_name', "~/audio/channel_2")
        self.declare_parameter('ros/topic_audio_channel_3_name', "~/audio/channel_3")
        self.declare_parameter('ros/topic_audio_channel_4_name', "~/audio/channel_4")
        self.declare_parameter('ros/topic_audio_channel_5_name', "~/audio/channel_5")
        topic_audio_channel_names = {
            0 : self.get_parameter('ros/topic_audio_channel_0_name').get_parameter_value().string_value,
            1 : self.get_parameter('ros/topic_audio_channel_1_name').get_parameter_value().string_value,
            2 : self.get_parameter('ros/topic_audio_channel_2_name').get_parameter_value().string_value,
            3 : self.get_parameter('ros/topic_audio_channel_3_name').get_parameter_value().string_value,
            4 : self.get_parameter('ros/topic_audio_channel_4_name').get_parameter_value().string_value,
            5 : self.get_parameter('ros/topic_audio_channel_5_name').get_parameter_value().string_value,
        }

        self.declare_parameter('ros/main_channel', 0)
        self._main_channel = self.get_parameter('ros/main_channel').get_parameter_value().integer_value

        # self.declare_parameter('ros/topic_doa_name', "~/doa")
        # self.topic_doa_name = self.get_parameter('ros/topic_doa_name').get_parameter_value()



        # default_brightness = rospy.get_param("~led/brightness", 10)
        # A_color = rospy.get_param("~led/A_color", [0,0,255])
        # B_color = rospy.get_param("~led/B_color", [0,255,0])

        # srv_SetBrightnessLED_name = rospy.get_param("~ros/srv_SetBrightnessLED_name", "~SetBrightnessLED")
        # srv_SetColorAllLED_name = rospy.get_param("~ros/srv_SetColorAllLED_name", "~SetColorAllLED")
        # srv_SetColorPaletteLED_name = rospy.get_param("~ros/srv_SetColorPaletteLED_name", "~SetColorPaletteLED")
        # srv_SetModeLED_name = rospy.get_param("~ros/srv_SetModeLED_name", "~SetModeLED")
        # topic_SetColorManualLED_name = rospy.get_param("~ros/topic_SetColorManualLED_name", "~SetColorManualLED")
        # mode = rospy.get_param("~led/mode", 1)



        # self._doa_yaw_offset = rospy.get_param("~doa_yaw_offset", 0)

        # # INIT dev to get DOA, set params, control PixelRing
        # self.dev = usb.core.find(idVendor=vendor_id,
        #                          idProduct=product_id)
        # if not self.dev:
        #     self.get_logger().error(f"Can`t find dev=respeaker by vendor_id ({vendor_id}) and product_id ({product_id})")

        # self.pixel_ring = PixelRing(self.dev)

        # try:
        #     self.pixel_ring.set_vad_led(1)
        # except usb.USBError as err:
        #     rospy.logerr(f"error in dev: {err}\nReset usb respeaker...")
        #     self.dev.reset()
        #     rospy.sleep(reset_time_sleep)
        # self.pixel_ring.set_vad_led(2)

        # self.pixel_ring.set_brightness(default_brightness)
        # self.pixel_ring.set_color_palette(A_color[0],A_color[1],A_color[2],B_color[0],B_color[1],B_color[2])
        # self.set_mode_led(mode)

        # INIT respeaker as audio input
        self.available_channels = None
        device_index = None
        with ignore_stderr(enable=True):
            self.pyaudio = pyaudio.PyAudio()

        count = self.pyaudio.get_device_count()
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info["name"]
            maxInputChannels = info["maxInputChannels"]

            if name.lower().find("respeaker") >= 0:
                self.available_channels = maxInputChannels
                device_index = i
                break

        if device_index is None:
            self.get_logger().error(f"respeaker_driver: Failed to find audio respeaker device by name")
            
        if self.available_channels != 6:
            self.get_logger().error(f"{self.available_channels} channel is found for audio respeaker device, but need 6\nYou may have to update firmware of respeaker.")

        try:
            self.stream = self.pyaudio.open(
                input=True, start=False,
                format=pyaudio.paInt16,
                channels=self.available_channels,
                rate=rate,
                frames_per_buffer=chunk,
                stream_callback=self._stream_callback,
                input_device_index=device_index,
            )
        except BaseException as err:
            self.get_logger().error(f"error in stream: {err}\nReset usb respeaker...")

            self.dev.reset()
            # rospy.sleep(reset_timeout)
            time.sleep(reset_timeout)
            self.stream = self.pyaudio.open(
                input=True, start=False,
                format=pyaudio.paInt16,
                channels=self.available_channels,
                rate=rate,
                frames_per_buffer=chunk,
                stream_callback=self._stream_callback,
                input_device_index=device_index,
            )

        # INIT ROS subjects
        self.publisher_audio_main = self.create_publisher(AudioData, topic_audio_main_name, 10)
        self.publishers_audio_channel = {c: self.create_publisher(AudioData, topic_audio_channel_names[c], 10) for c in range(self.available_channels)}

        # self.pub_doa = rospy.Publisher(topic_doa_angle_name, Int16, queue_size=10)

        # rospy.Service(srv_SetBrightnessLED_name, SetBrightnessLED, self._SetBrightnessLED_callback)
        # rospy.Service(srv_SetColorAllLED_name, SetColorAllLED, self._SetColorAllLED_callback)
        # rospy.Service(srv_SetColorPaletteLED_name, SetColorPaletteLED, self._SetColorPaletteLED_callback)
        # rospy.Service(srv_SetModeLED_name, SetModeLED, self._SetModeLED_callback)
        # rospy.Subscriber(topic_SetColorManualLED_name, SetColorManualLED, callback=self._SetColorManualLED_callback)
        
        self.prev_doa = 400 # просто стартовое значение

        self.get_logger().info("respeaker_driver 123 123 INITED")

    # def _SetColorManualLED_callback(self, msg:SetColorManualLED):
    #     isCorrect = 1
    #     new_data = [0]*36
    #     for i in range(12):
    #         r = int(msg.colors[i].r)
    #         g = int(msg.colors[i].g)
    #         b = int(msg.colors[i].b)
    #         if (0<=r<=255) and (0<=g<=255) and (0<=b<=255):
                
    #             new_data[3*i+0] = r
    #             new_data[3*i+1] = g
    #             new_data[3*i+2] = b
    #         else:
    #             isCorrect = 0
    #             break
    #     if isCorrect:
    #         self.pixel_ring.set_color_manual(data=new_data)

    # def _SetBrightnessLED_callback(self, request:SetBrightnessLEDRequest):
    #     response = SetBrightnessLEDResponse()
    #     if 0<=request.brightness<=31:
    #         self.pixel_ring.set_brightness(request.brightness)
    #         response.value = 0
    #     else:
    #         response.value = -1
    #     return response

    # def _SetColorAllLED_callback(self, request:SetColorAllLEDRequest):
    #     response = SetColorAllLEDResponse()

    #     if (0<=request.r<=255) and (0<=request.g<=255) and (0<=request.b<=255):
    #         self.pixel_ring.set_color_all(r=request.r,g=request.g,b=request.b)
    #         response.value = 0
    #     else:
    #         response.value = -1
    #     return response

    # def _SetColorPaletteLED_callback(self, request:SetColorPaletteLEDRequest):
    #     response = SetColorPaletteLEDResponse()

    #     if (0<=request.colorA[0]<=255) and (0<=request.colorA[1]<=255) and (0<=request.colorA[2]<=255) and (0<=request.colorB[0]<=255) and (0<=request.colorB[0]<=255) and (0<=request.colorB[0]<=255):
    #         self.pixel_ring.set_color_palette(request.colorA[0],request.colorA[1],request.colorA[2],
    #                                           request.colorB[0],request.colorB[1],request.colorB[2])
    #         response.value = 0
    #     else:
    #         response.value = -1
    #     return response

    # def _SetModeLED_callback(self, request:SetModeLEDRequest):
    #     response = SetModeLEDResponse()

    #     if 0 <= request.mode <= 5:
    #         self.set_mode_led(request.mode)
    #         response.value = 0
    #     else:
    #         response.value = -1
    #     return response

    # def set_mode_led(self, mode:int):
    #     if mode==0:
    #         self.pixel_ring.off()
    #     elif mode==1:
    #         self.pixel_ring.trace()
    #     elif mode==2:
    #         self.pixel_ring.listen()
    #     elif mode==3:
    #         self.pixel_ring.wait()
    #     elif mode==4:
    #         self.pixel_ring.speak()
    #     elif mode==5:
    #         self.pixel_ring.spin()

    def __del__(self):
        try:
            usb.util.dispose_resources(self.dev)
            self.dev.close()
        except:
            pass
        finally:
            self.dev = None

        try:
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None

        try:
            self.pyaudio.terminate()
        except:
            pass

    def start_stream(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop_stream(self):
        if self.stream.is_active():
            self.stream.stop_stream()
    
    # def get_doa_angle(self):
    #     doa = self._read('DOAANGLE')
    #     rad = math.radians(doa)
    #     x = math.cos(rad)
    #     y = math.sin(rad)
    #     rotate = math.radians(self._doa_yaw_offset)
    #     x_ = x*math.cos(rotate)+y*math.sin(rotate)
    #     y_ = -x*math.sin(rotate)+y*math.cos(rotate)
    #     doa_angle = int(math.degrees(math.atan2(y_, x_)))
    #     return -doa_angle

    def _stream_callback(self, in_data, frame_count, time_info, status):
        # doa = self.get_doa_angle()
        # if self.prev_doa != doa:
        #     self.prev_doa = doa
        #     self.pub_doa.publish(doa)

        data = np.frombuffer(buffer=in_data, dtype=np.int16)
        chunk_per_channel = len(data) / self.available_channels
        data = np.reshape(data, (int(chunk_per_channel), self.available_channels))

        for channel in range(self.available_channels):
            channel_data = data[:, channel]

            msg = AudioData()
            msg.data = channel_data.tostring()
            self.publishers_audio_channel[channel].publish(msg)
            if channel == self._main_channel:
                self.publisher_audio_main.publish(msg)

        return None, pyaudio.paContinue

    def _write(self, name:str, value:float):
        try:
            data = RESPEAKER_PARAMETERS[name]
        except KeyError:
            rospy.logerr(f"Parameter {name} was not found")
            return

        if data[5] == 'ro':
            rospy.loginfo(f'Parameter {name} is read-only')

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self._timeout)

    def _read(self, name:str):
        try:
            data = RESPEAKER_PARAMETERS[name]
        except KeyError:
            rospy.logerr(f"Parameter {name} was not found")
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self._timeout)

        response = struct.unpack(b'ii', response.tobytes())

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])

        return result

def main(args=None):
    rclpy.init(args=None)

    respeaker_driver = RespeakerDriver()
    respeaker_driver.start_stream()

    rclpy.spin(respeaker_driver)

    respeaker_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
