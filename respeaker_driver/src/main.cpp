#include <rclcpp/rclcpp.hpp>
#include <robohead_interfaces/msg/audio_data.hpp>
#include <robohead_interfaces/srv/simple_command.hpp>
#include <robohead_interfaces/msg/color.hpp>
#include <robohead_interfaces/msg/color_array.hpp>
#include <robohead_interfaces/srv/color_palette.hpp>
#include <robohead_interfaces/srv/color.hpp>

#define PA_NO_JACK
#include <portaudio.h>
#include <alsa/asoundlib.h>
#include <vector>
#include <string>
#include <libusb-1.0/libusb.h>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>

/* example usage
ros2 service call /respeaker_driver/set_color_all robohead_interfaces/srv/Color "red: 0
green: 0
blue: 255" 
*/

/*
ros2 service call /respeaker_driver/set_mode robohead_interfaces/srv/SimpleCommand "data: 4"
*/

/* example usage
ros2 service call /respeaker_driver/set_color_palette robohead_interfaces/srv/ColorPalette "{
  color_a: {red: 255, green: 0, blue: 0},
  color_b: {red: 0, green: 0, blue: 255}
}"
*/

/*
ros2 topic pub /respeaker_driver/set_color_manual robohead_interfaces/msg/ColorArray "{
  colors: [
    {red: 255, green: 0, blue: 0},
    {red: 0, green: 255, blue: 0},
    {red: 0, green: 0, blue: 255},

    {red: 255, green: 0, blue: 0},
    {red: 0, green: 255, blue: 0},
    {red: 0, green: 0, blue: 255},

    {red: 255, green: 0, blue: 0},
    {red: 0, green: 255, blue: 0},
    {red: 0, green: 0, blue: 255},

    {red: 255, green: 0, blue: 0},
    {red: 0, green: 255, blue: 0},
    {red: 0, green: 0, blue: 255}
  ]
}" --once
*/

class RespeakerDriver : public rclcpp::Node

{
    libusb_context *usb_ctx_ = nullptr;
    libusb_device_handle *usb_dev_ = nullptr;
    std::mutex usb_mutex_;

    int usb_vendor_id_, usb_product_id_, usb_timeout_, usb_sleep_reset_, usb_sleep_stop_;
    int audio_sample_rate_, audio_frames_per_buffer_, audio_count_of_channels_, audio_main_channel_;

    // PortAudio
    PaStream *stream_ = nullptr;
    int device_index_ = -1;
    int num_channels_ = 6;

    // ROS2
    rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr pub_main_;
    std::vector<rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr> pub_channels_;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_set_mode_;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_set_brightness_;
    rclcpp::Service<robohead_interfaces::srv::Color>::SharedPtr srv_set_color_all_;
    rclcpp::Service<robohead_interfaces::srv::ColorPalette>::SharedPtr srv_set_color_palette_;
    rclcpp::Subscription<robohead_interfaces::msg::ColorArray>::SharedPtr sub_set_color_manual_;

public:
    RespeakerDriver() : Node("respeaker_driver")
  {
      // Declare Parameters
      this->declare_parameter("usb/vendor_id", 0x2886);
      this->declare_parameter("usb/product_id", 0x0018);
      this->declare_parameter("usb/timeout", 5000);
      this->declare_parameter("usb/sleep_reset", 500);
      this->declare_parameter("usb/sleep_stop", 100);

      this->declare_parameter("audio/sample_rate", 16000);
      this->declare_parameter("audio/frames_per_buffer", 1024);
      this->declare_parameter("audio/count_of_channels", 6);
      this->declare_parameter("audio/main_channel", 0);

      this->declare_parameter("ros/topic_name/audio_main", "~/audio/main");
      this->declare_parameter("ros/topic_name/audio_channel_0", "~/audio/channel_0");
      this->declare_parameter("ros/topic_name/audio_channel_1", "~/audio/channel_1");
      this->declare_parameter("ros/topic_name/audio_channel_2", "~/audio/channel_2");
      this->declare_parameter("ros/topic_name/audio_channel_3", "~/audio/channel_3");
      this->declare_parameter("ros/topic_name/audio_channel_4", "~/audio/channel_4");
      this->declare_parameter("ros/topic_name/audio_channel_5", "~/audio/channel_5");
      this->declare_parameter("ros/topic_name/doa", "~/doa");
      this->declare_parameter("ros/topic_name/set_color_manual", "~/set_color_manual");

      this->declare_parameter("ros/service_name/set_brightness", "~/set_brightness");
      this->declare_parameter("ros/service_name/set_color_all", "~/set_color_all");
      this->declare_parameter("ros/service_name/set_color_palette", "~/set_color_palette");
      this->declare_parameter("ros/service_name/set_mode", "~/set_mode");

    // get parameters

    this->get_parameter("usb/vendor_id", usb_vendor_id_);
      this->get_parameter("usb/product_id", usb_product_id_);
      this->get_parameter("usb/timeout", usb_timeout_);
      this->get_parameter("usb/sleep_reset", usb_sleep_reset_);
      this->get_parameter("usb/sleep_stop", usb_sleep_stop_);

      this->get_parameter("audio/sample_rate", audio_sample_rate_);
      this->get_parameter("audio/frames_per_buffer", audio_frames_per_buffer_);
      this->get_parameter("audio/count_of_channels", audio_count_of_channels_);
      this->get_parameter("audio/main_channel", audio_main_channel_);

      std::string topic_name_audio_main = this->get_parameter("ros/topic_name/audio_main").as_string();
      std::vector<std::string> topic_names_channel;
      topic_names_channel.push_back(this->get_parameter("ros/topic_name/audio_channel_0").as_string());
      topic_names_channel.push_back(this->get_parameter("ros/topic_name/audio_channel_1").as_string());
      topic_names_channel.push_back(this->get_parameter("ros/topic_name/audio_channel_2").as_string());
      topic_names_channel.push_back(this->get_parameter("ros/topic_name/audio_channel_3").as_string());
      topic_names_channel.push_back(this->get_parameter("ros/topic_name/audio_channel_4").as_string());
      topic_names_channel.push_back(this->get_parameter("ros/topic_name/audio_channel_5").as_string());
      std::string topic_name_doa = this->get_parameter("ros/topic_name/doa").as_string();
      std::string topic_name_set_color_manual = this->get_parameter("ros/topic_name/set_color_manual").as_string();

      std::string service_name_set_brightness = this->get_parameter("ros/service_name/set_brightness").as_string();
      std::string service_name_set_color_all = this->get_parameter("ros/service_name/set_color_all").as_string();
      std::string service_name_set_color_palette = this->get_parameter("ros/service_name/set_color_palette").as_string();
      std::string service_name_set_set_mode = this->get_parameter("ros/service_name/set_mode").as_string();

    if (!initUsb()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize USB");
        return;
    }

      // Инициализация PortAudio
      PaError err = Pa_Initialize();

      if (err != paNoError) {
          RCLCPP_ERROR(this->get_logger(), "PortAudio init failed: %s", Pa_GetErrorText(err));
      }

      // Поиск ReSpeaker
      int numDevices = Pa_GetDeviceCount();
      for (int i = 0; i < numDevices; i++) {
          const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
          if (info && info->name && std::string(info->name).find("ReSpeaker") != std::string::npos) {
              device_index_ = i;
              num_channels_ = info->maxInputChannels;
              RCLCPP_INFO(this->get_logger(), "Found ReSpeaker: %s (%d channels)", info->name, num_channels_);
              break;
          }
      }

    if (device_index_ == -1) {
        RCLCPP_WARN(this->get_logger(), "ReSpeaker not found on first attempt. Trying USB reset...");

        // Сброс устройства по VID/PID
        if (resetUsbDevice(usb_vendor_id_, usb_product_id_)) {
            // Дадим системе немного времени на перечисление
            std::this_thread::sleep_for(std::chrono::milliseconds(usb_sleep_reset_));
            Pa_Terminate();

            err = Pa_Initialize();


            if (err != paNoError) {
                RCLCPP_ERROR(this->get_logger(), "PortAudio re-init failed: %s", Pa_GetErrorText(err));
                return;
            }
            // Повторный поиск
            int numDevices = Pa_GetDeviceCount();
            for (int i = 0; i < numDevices; i++) {
                const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
                if (info && info->name && std::string(info->name).find("ReSpeaker") != std::string::npos) {
                    device_index_ = i;
                    num_channels_ = info->maxInputChannels;
                    RCLCPP_INFO(this->get_logger(), "ReSpeaker found after USB reset: %s (%d channels)", info->name, num_channels_);
                    break;
                }
            }
        }
    }

      if (device_index_ == -1) {
          RCLCPP_ERROR(this->get_logger(), "ReSpeaker not found! Check: arecord -l");
          Pa_Terminate();
          return;
      }

      if (num_channels_ != audio_count_of_channels_) { 
          RCLCPP_WARN(this->get_logger(), "Expected %d channels, got %d", audio_count_of_channels_, num_channels_);
      }
      if (audio_main_channel_ < 0 || audio_main_channel_ >= audio_count_of_channels_) { 
          RCLCPP_WARN(this->get_logger(), "Main channel: %d. It`s out of %d channels", audio_main_channel_, audio_count_of_channels_);
      }

      // Создание публикаторов
      RCLCPP_INFO(this->get_logger(), "Start create pubs");
      pub_main_ = this->create_publisher<robohead_interfaces::msg::AudioData>(topic_name_audio_main, 10);
      for (int i = 0; i < num_channels_; i++) {
          rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr pub = this->create_publisher<robohead_interfaces::msg::AudioData>(
              topic_names_channel[i], 10);
          pub_channels_.push_back(pub);
      }
      RCLCPP_INFO(this->get_logger(), "Try open stream");

            // Открытие потока
      if (!initAudio()) {
          // Освобождаем USB
          if (usb_dev_) {
              libusb_release_interface(usb_dev_, 0);
              libusb_close(usb_dev_);
              usb_dev_ = nullptr;
          }
          if (usb_ctx_) {
              libusb_exit(usb_ctx_);
              usb_ctx_ = nullptr;
          }
          return;
      }

    srv_set_mode_ = this->create_service<robohead_interfaces::srv::SimpleCommand>(
        service_name_set_set_mode,
        std::bind(&RespeakerDriver::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_brightness_ = this->create_service<robohead_interfaces::srv::SimpleCommand>(
        service_name_set_brightness,
    std::bind(&RespeakerDriver::setBrightnessCallback, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_color_all_ = this->create_service<robohead_interfaces::srv::Color>(
        service_name_set_color_all,
        std::bind(&RespeakerDriver::setColorAllCallback, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_color_palette_ = this->create_service<robohead_interfaces::srv::ColorPalette>(
        service_name_set_color_palette,
        std::bind(&RespeakerDriver::setColorPaletteCallback, this, std::placeholders::_1, std::placeholders::_2));

    sub_set_color_manual_ = this->create_subscription<robohead_interfaces::msg::ColorArray>(
        topic_name_set_color_manual, 10,
        std::bind(&RespeakerDriver::setColorManualCallback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "respeaker_driver INITED");

  }

    ~RespeakerDriver()
    {
        // 1. Остановка и закрытие аудиопотока
        if (stream_) {
            Pa_AbortStream(stream_);
            Pa_CloseStream(stream_);
            stream_ = nullptr;
        }

        // 2. Завершаем PortAudio СРАЗУ — освобождаем ALSA
        Pa_Terminate();

        std::this_thread::sleep_for(std::chrono::milliseconds(usb_sleep_stop_));

        // 3. Теперь безопасно работать с USB
        if (usb_dev_) {
            // Попытка вернуть ядерный драйвер
            if (libusb_kernel_driver_active(usb_dev_, 0) == 0) {
                int err = libusb_attach_kernel_driver(usb_dev_, 0);
                if (err == LIBUSB_SUCCESS) {
                    RCLCPP_DEBUG(this->get_logger(), "Kernel driver successfully reattached");
                } else if (err != LIBUSB_ERROR_NOT_FOUND) {
                    // LIBUSB_ERROR_BUSY — нормально, если ALSA ещё не отпустила
                    RCLCPP_DEBUG(this->get_logger(), "Could not reattach kernel driver: %s", libusb_error_name(err));
                }
            }

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

bool resetUsbDevice(uint16_t vid, uint16_t pid)
{
    libusb_context* ctx = nullptr;
    int err = libusb_init(&ctx);
    if (err < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("respeaker_driver"), "Failed to init libusb for reset: %d", err);
        return false;
    }

    libusb_device_handle* dev = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (!dev) {
        RCLCPP_WARN(rclcpp::get_logger("respeaker_driver"), "Device not found for reset (VID:0x%04x, PID:0x%04x)", vid, pid);
        libusb_exit(ctx);
        return false;
    }

    // Отвязываем ядерный драйвер, если он активен
    if (libusb_kernel_driver_active(dev, 0) == 1) {
        err = libusb_detach_kernel_driver(dev, 0);
        if (err != LIBUSB_SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger("respeaker_driver"), "Failed to detach kernel driver: %s", libusb_error_name(err));
            // Продолжаем попытку сброса даже без отвязки
        }
    }

    // Теперь сбрасываем
    err = libusb_reset_device(dev);
    if (err != LIBUSB_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("respeaker_driver"), "Failed to reset device: %s", libusb_error_name(err));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("respeaker_driver"), "Successfully reset USB device");
    }

    // Возвращаем ядру управление (опционально, но хорошо для чистоты)
    if (libusb_kernel_driver_active(dev, 0) == 0) {
        libusb_attach_kernel_driver(dev, 0);
    }

    libusb_close(dev);
    libusb_exit(ctx);
    return (err == LIBUSB_SUCCESS);
}



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
                response->data = writeCmd(0x01, {0, 0, 0, 0});; // set color all 0,0,0
                break;
            case 1: // trace
                response->data = writeCmd(0);
                break;
            case 2: // listen
                response->data = writeCmd(2);
                break;
            case 3: // wait
                response->data = writeCmd(3);
                break;
            case 4: // speak
                response->data = writeCmd(4);
                break; 
            case 5: // spin
                response->data = writeCmd(5);
                break;
        }
    }


    void setColorAllCallback(
        const std::shared_ptr<robohead_interfaces::srv::Color::Request> request,
        std::shared_ptr<robohead_interfaces::srv::Color::Response> response)
    {
        uint8_t r = request->red;
        uint8_t g = request->green;
        uint8_t b = request->blue;
        response->data = writeCmd(0x01, {r, g, b, 0});
    }

    void setColorPaletteCallback(
        const std::shared_ptr<robohead_interfaces::srv::ColorPalette::Request> request,
        std::shared_ptr<robohead_interfaces::srv::ColorPalette::Response> response)
    {
        auto& a = request->color_a;
        auto& b = request->color_b;
        response->data = -1;

        response->data = writeCmd(0x21, {
                                        a.red, a.green, a.blue, 0,
                                        b.red, b.green, b.blue, 0
                                        });
    }

    void setColorManualCallback(
        const robohead_interfaces::msg::ColorArray::SharedPtr msg)
    {
        if (msg->colors.size() != 12) {
            RCLCPP_WARN(this->get_logger(), "Expected 12 colors, got %zu", msg->colors.size());
            return;
        }

        std::vector<uint8_t> data(48, 0);
        for (size_t i = 0; i<12; i++)
        {
            data[i*4]   = msg->colors[i].red;
            data[i*4+1] = msg->colors[i].green;
            data[i*4+2] = msg->colors[i].blue;
            data[i*4+3] = 0;
        }

        writeCmd(0x06, data);
    }


    bool initUsb()
    {
        int err = libusb_init(&usb_ctx_);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "libusb init failed: %d", err);
            return false;
        }

        usb_dev_ = libusb_open_device_with_vid_pid(usb_ctx_, usb_vendor_id_, usb_product_id_);
        if (!usb_dev_) {
            RCLCPP_ERROR(this->get_logger(), "ReSpeaker not found (VID: 0x%04x, PID: 0x%04x)", usb_vendor_id_, usb_product_id_);
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
      inputParams.device = device_index_;
      inputParams.channelCount = num_channels_;
      inputParams.sampleFormat = paInt16;
      inputParams.suggestedLatency = Pa_GetDeviceInfo(device_index_)->defaultLowInputLatency;
      inputParams.hostApiSpecificStreamInfo = nullptr;

      PaError err = Pa_OpenStream(
          &stream_,
          &inputParams,
          nullptr, // output
          audio_sample_rate_,
          audio_frames_per_buffer_,
          paClipOff,
          audioCallback,
          this
      );

      if (err != paNoError) {
          RCLCPP_ERROR(this->get_logger(), "Open stream failed: %s", Pa_GetErrorText(err));
          return false;
      }

      err = Pa_StartStream(stream_);
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
    const int num_channels = node->num_channels_;

    // Публикуем каждый канал
    for (int ch = 0; ch < num_channels; ++ch) {
        robohead_interfaces::msg::AudioData msg;
        msg.data.resize(frameCount);  // frameCount сэмплов типа int16

        for (size_t i = 0; i < frameCount; ++i) {
            msg.data[i] = data[i * num_channels + ch];
        }

        node->pub_channels_[ch]->publish(std::move(msg));
    }

    // Публикуем основной канал отдельно (если нужно дублировать)
    if (node->audio_main_channel_ >= 0 && node->audio_main_channel_ < num_channels) {
        robohead_interfaces::msg::AudioData main_msg;
        main_msg.data.resize(frameCount);
        for (size_t i = 0; i < frameCount; ++i) {
            main_msg.data[i] = data[i * num_channels + node->audio_main_channel_];
        }
        node->pub_main_->publish(std::move(main_msg));
    }

    return paContinue;
}

};


static void alsa_error_handler(const char *file, int line, const char *function,
                               int err, const char *fmt, ...) {
    // Пусто — подавляем все ALSA-предупреждения
    (void)file; (void)line; (void)function; (void)err; (void)fmt;
    // Аргументы ... игнорируем — нам не нужны сообщения
}

int main(int argc, char *argv[])
{

    //     // Подавляем ALSA-предупреждения
    snd_lib_error_set_handler(alsa_error_handler);

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<RespeakerDriver>();

    // Используем многопоточный исполнитель для безопасной публикации из колбэка
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}