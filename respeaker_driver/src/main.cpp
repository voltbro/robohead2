#include <rclcpp/rclcpp.hpp>
#include <robohead_interfaces/msg/audio_data.hpp>
#include "robohead_interfaces/srv/simple_command.hpp" 
#include <portaudio.h>
#include <vector>
#include <string>
#include <libusb-1.0/libusb.h>
#include <cstring>
#include <memory>
#include <mutex>

class RespeakerDriver : public rclcpp::Node

{

    // Параметры
    int sample_rate;
    int frames_per_buffer;
    int main_channel;

    // PortAudio
    PaStream *stream = nullptr;
    int device_index = -1;
    int num_channels = 6;

    // ROS2
    rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr pub_main;
    std::vector<rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr> pub_channels;
    std::string topic_audio_main_name;
    std::vector<std::string> topic_audio_channel_names;

    libusb_context *usb_ctx_ = nullptr;
    libusb_device_handle *usb_dev_ = nullptr;
    std::mutex usb_mutex_;
    int usb_timeout_ = 5000;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_mode_;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_brightness_;

public:
    RespeakerDriver() : Node("respeaker_driver")
  {
      // Параметр
      this->declare_parameter("audio/sample_rate", 16000);
      this->declare_parameter("audio/frames_per_buffer", 1024);
      this->declare_parameter("ros/main_channel", 0);
      this->declare_parameter("ros/topic_audio_main_name", "~audio/main");
      this->declare_parameter("ros/topic_audio_channel_0_name", "~audio/channel_0");
      this->declare_parameter("ros/topic_audio_channel_1_name", "~audio/channel_1");
      this->declare_parameter("ros/topic_audio_channel_2_name", "~audio/channel_2");
      this->declare_parameter("ros/topic_audio_channel_3_name", "~audio/channel_3");
      this->declare_parameter("ros/topic_audio_channel_4_name", "~audio/channel_4");
      this->declare_parameter("ros/topic_audio_channel_5_name", "~audio/channel_5");


      this->get_parameter("audio/sample_rate", sample_rate);
      this->get_parameter("audio/frames_per_buffer", frames_per_buffer);
      this->get_parameter("ros/main_channel", main_channel);
      this->get_parameter("ros/topic_audio_main_name", topic_audio_main_name);
    //   this->get_parameter("ros/topic_audio_channel_0_name", topic_audio_channel_names);
    //   this->get_parameter("ros/topic_audio_channel_1_name", topic_audio_channel_names);
    //   this->get_parameter("ros/topic_audio_channel_2_name", topic_audio_channel_names);
    //   this->get_parameter("ros/topic_audio_channel_3_name", topic_audio_channel_names);
    //   this->get_parameter("ros/topic_audio_channel_4_name", topic_audio_channel_names);
    //   this->get_parameter("ros/topic_audio_channel_5_name", topic_audio_channel_names);




      // Инициализация PortAudio
      PaError err = Pa_Initialize();
      if (err != paNoError) {
          RCLCPP_ERROR(this->get_logger(), "PortAudio init failed: %s", Pa_GetErrorText(err));
          return;
      }

      // Поиск ReSpeaker
      int numDevices = Pa_GetDeviceCount();
      for (int i = 0; i < numDevices; i++) {
          const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
          if (info && info->name && std::string(info->name).find("ReSpeaker") != std::string::npos) {
              device_index = i;
              num_channels = info->maxInputChannels;
              RCLCPP_INFO(this->get_logger(), "Found ReSpeaker: %s (%d channels)", info->name, num_channels);
              break;
          }
      }

      if (device_index == -1) {
          RCLCPP_ERROR(this->get_logger(), "ReSpeaker not found! Check: arecord -l");
          Pa_Terminate();
          return;
      }

      if (num_channels != 6) {
          RCLCPP_WARN(this->get_logger(), "Expected 6 channels, got %d", num_channels);
      }

      // Создание публикаторов
      RCLCPP_INFO(this->get_logger(), "Start create pubs");
      pub_main = this->create_publisher<robohead_interfaces::msg::AudioData>("~/audio/main", 10);
      for (int i = 0; i < num_channels; i++) {
          rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr pub = this->create_publisher<robohead_interfaces::msg::AudioData>(
              "~/audio/channel_" + std::to_string(i), 10);
          pub_channels.push_back(pub);
      }
      RCLCPP_INFO(this->get_logger(), "Try open stream");

      // Открытие потока
      if (!initAudio()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream");
          Pa_Terminate();
      }


    if (!initUsb()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize USB");
        return;
    }

    srv_mode_ = this->create_service<robohead_interfaces::srv::SimpleCommand>(
        "~/set_mode",
        std::bind(&RespeakerDriver::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    srv_brightness_ = this->create_service<robohead_interfaces::srv::SimpleCommand>(
        "~/set_brightness",
    std::bind(&RespeakerDriver::setBrightnessCallback, this, std::placeholders::_1, std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "respeaker_driver INITED");

  }


  ~RespeakerDriver()
  {
    if (stream) {
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        stream = nullptr;
    }
    Pa_Terminate();


    if (usb_dev_) {
        libusb_release_interface(usb_dev_, 0);
        libusb_close(usb_dev_);
        usb_dev_ = nullptr;
    }
    if (usb_ctx_) {
        libusb_exit(usb_ctx_);
        usb_ctx_ = nullptr;
    }
  }

  private:


    int writeCmd(uint8_t cmd, const std::vector<uint8_t>& data={0})
    {
        std::lock_guard<std::mutex> lock(usb_mutex_);
        if (!usb_dev_) return -1;

        int err = libusb_control_transfer(
            usb_dev_,
            LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
            0x00,
            cmd,
            0x1C,
            const_cast<uint8_t*>(data.data()),
            data.size(),
            usb_timeout_
        );

        return (err == static_cast<int>(data.size())) ? 0 : -1;
    }

    void setBrightnessCallback(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        int brightness = request->data;
        response->data = -1;

        if (brightness >= 0 && brightness <= 31) {
            response->data = writeCmd(0x20, {static_cast<uint8_t>(brightness)});
        }
    }

    void setModeCallback(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        response->data = -1;

        switch (request->data)
        {
            case 0: // off
                response->data = writeCmd(0); // set color all 0,0,0
                break;
            case 1: // trace
                response->data = writeCmd(0);
                break;
            case 2: // listen
                response->data = writeCmd(2);
                break;
            case 3: // spin
                response->data = writeCmd(3);
                break;
            case 4: // speak
                response->data = writeCmd(4);
                break; 
            case 5: // wait
                response->data = writeCmd(5);
                break;
        }
    }

    bool initUsb()
    {
        int err = libusb_init(&usb_ctx_);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "libusb init failed: %d", err);
            return false;
        }

        usb_dev_ = libusb_open_device_with_vid_pid(usb_ctx_, 0x2886, 0x0018);
        if (!usb_dev_) {
            RCLCPP_ERROR(this->get_logger(), "ReSpeaker not found (VID: 0x%04x, PID: 0x%04x)", 0x2886, 0x0018);
            return false;
        }

        if (libusb_kernel_driver_active(usb_dev_, 0) == 1) {
            libusb_detach_kernel_driver(usb_dev_, 0);
        }

        err = libusb_claim_interface(usb_dev_, 0);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "libusb claim interface failed: %d", err);
            return false;
        }

        return true;
    }

  bool initAudio()
  {
      PaStreamParameters inputParams;
      inputParams.device = device_index;
      inputParams.channelCount = num_channels;
      inputParams.sampleFormat = paInt16;
      inputParams.suggestedLatency = Pa_GetDeviceInfo(device_index)->defaultLowInputLatency;
      inputParams.hostApiSpecificStreamInfo = nullptr;

      PaError err = Pa_OpenStream(
          &stream,
          &inputParams,
          nullptr, // output
          sample_rate,
          frames_per_buffer,
          paClipOff,
          audioCallback,
          this
      );

      if (err != paNoError) {
          RCLCPP_ERROR(this->get_logger(), "Open stream failed: %s", Pa_GetErrorText(err));
          return false;
      }

      err = Pa_StartStream(stream);
      if (err != paNoError) {
          RCLCPP_ERROR(this->get_logger(), "Start stream failed: %s", Pa_GetErrorText(err));
          return false;
      }

      return true;
  }


  static int audioCallback(
      const void *input, void *output,
      unsigned long frameCount,
      const PaStreamCallbackTimeInfo* timeInfo,
      PaStreamCallbackFlags statusFlags,
      void *userData)
  {
      (void)output; (void)timeInfo; (void)statusFlags;

      RespeakerDriver* node = static_cast<RespeakerDriver*>(userData);
      const int16_t* data = static_cast<const int16_t*>(input);

      robohead_interfaces::msg::AudioData msg;
      msg.data.resize(frameCount * sizeof(int16_t));

      for (int ch = 0; ch < node->num_channels; ch++) {
          int16_t* out = reinterpret_cast<int16_t*>(msg.data.data());
          const int16_t* in = &data[ch];
          for (size_t i = 0; i < frameCount; i++) {
              out[i] = in[i * node->num_channels];
          }

          node->pub_channels[ch]->publish(msg);
          if (ch == node->main_channel) {
              node->pub_main->publish(msg);
          }
      }

      return paContinue;
  }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<RespeakerDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}