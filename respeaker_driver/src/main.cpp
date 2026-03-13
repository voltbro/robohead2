#include <rclcpp/rclcpp.hpp>
#include <robohead_interfaces/msg/audio_data.hpp>
#include <robohead_interfaces/srv/simple_command.hpp>
#include <robohead_interfaces/msg/color.hpp>
#include <robohead_interfaces/msg/color_array.hpp>
#include <robohead_interfaces/srv/color_palette.hpp>
#include <robohead_interfaces/srv/color.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <alsa/asoundlib.h>

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <functional>

#include "usb_handler.hpp"
#include "audio_handler.hpp"

// ================================================================
//  ROS2 Node
// ================================================================

class RespeakerDriver : public rclcpp::Node
{
public:
    RespeakerDriver()
        : Node("respeaker_driver")
        , actual_channels_(0)
        , doa_yaw_offset_rad_(0.0)
        , usb_vid_(0)
        , usb_pid_(0)
        , usb_timeout_(5000)
        , usb_sleep_reset_(500)
        , usb_sleep_stop_(100)
    {
        declareParameters();

        if (!initUsb())
            return;

        if (!initAudio())
        {
            usb_.reset();
            return;
        }

        createRosInterfaces();

        RCLCPP_INFO(get_logger(), "INITED [%s]  channels=%d",
                     detected_version_.c_str(), actual_channels_);
    }

    ~RespeakerDriver()
    {
        audio_.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(usb_sleep_stop_));
        usb_.reset();
    }

private:
    // ================================================================
    //  Параметры
    // ================================================================

    void declareParameters()
    {
        // USB
        usb_vid_         = static_cast<uint16_t>(
            this->declare_parameter<int>("usb.vendor_id", 0));
        usb_pid_         = static_cast<uint16_t>(
            this->declare_parameter<int>("usb.product_id", 0));
        usb_timeout_     = this->declare_parameter<int>("usb.timeout", 5000);
        usb_sleep_reset_ = this->declare_parameter<int>("usb.sleep_reset", 500);
        usb_sleep_stop_  = this->declare_parameter<int>("usb.sleep_stop", 100);

        // Audio
        audio_cfg_.sample_rate       = this->declare_parameter<int>("audio.sample_rate", 16000);
        audio_cfg_.frames_per_buffer = this->declare_parameter<int>("audio.frames_per_buffer", 1024);
        audio_cfg_.count_of_channels = this->declare_parameter<int>("audio.count_of_channels", 0);
        audio_cfg_.main_channel      = this->declare_parameter<int>("audio.main_channel", 0);

        audio_device_primary_  = this->declare_parameter<std::string>(
            "audio.device_name_primary", "ReSpeaker");
        audio_device_fallback_ = this->declare_parameter<std::string>(
            "audio.device_name_fallback", "Mic Array");

        // Topic names
        topic_audio_main_ = this->declare_parameter<std::string>(
            "ros.topic_name.audio_main", "audio/main");

        for (int i = 0; i < AudioHandler::MAX_CHANNELS; ++i)
        {
            std::string key = "ros.topic_name.audio_channel_" + std::to_string(i);
            std::string def = "audio/channel_" + std::to_string(i);
            channel_topics_.push_back(this->declare_parameter<std::string>(key, def));
        }

        topic_doa_          = this->declare_parameter<std::string>(
            "ros.topic_name.doa", "doa");
        topic_speech_       = this->declare_parameter<std::string>(
            "ros.topic_name.speech_detected", "speech_detected");
        topic_color_manual_ = this->declare_parameter<std::string>(
            "ros.topic_name.set_color_manual", "set_color_manual");

        // Service names
        srv_name_mode_       = this->declare_parameter<std::string>(
            "ros.service_name.set_mode", "set_mode");
        srv_name_brightness_ = this->declare_parameter<std::string>(
            "ros.service_name.set_brightness", "set_brightness");
        srv_name_color_all_  = this->declare_parameter<std::string>(
            "ros.service_name.set_color_all", "set_color_all");
        srv_name_palette_    = this->declare_parameter<std::string>(
            "ros.service_name.set_color_palette", "set_color_palette");

        // DOA offset
        doa_yaw_offset_rad_ =
            this->declare_parameter<double>("doa_yaw_offset", 0.0) * M_PI / 180.0;
    }

    // ================================================================
    //  USB init
    // ================================================================

    bool initUsb()
    {
        UsbHandler::DetectedDevice det =
            UsbHandler::autoDetect(usb_vid_, usb_pid_);

        if (!det.found)
        {
            RCLCPP_ERROR(get_logger(),
                         "No ReSpeaker found! VID=0x%04X PID=0x%04X (0=auto)",
                         usb_vid_, usb_pid_);
            return false;
        }

        detected_version_ = det.version;

        RCLCPP_INFO(get_logger(), "Detected: %s  VID=0x%04X  PID=0x%04X",
                     det.version.c_str(), det.vid, det.pid);

        usb_ = UsbHandler::create(det.version, det.vid, det.pid, usb_timeout_);

        if (!usb_)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create USB handler");
            return false;
        }

        if (!usb_->init())
        {
            RCLCPP_ERROR(get_logger(), "USB init failed");
            usb_.reset();
            return false;
        }

        return true;
    }

    // ================================================================
    //  Audio init
    // ================================================================

    bool initAudio()
    {
        audio_ = std::make_unique<AudioHandler>(this, audio_cfg_);

        // USB reset callback
        AudioHandler::ResetCallback reset_fn =
            std::bind(&RespeakerDriver::usbResetCallback, this);

        // Audio frame callback
        AudioHandler::FrameCallback frame_fn =
            std::bind(&RespeakerDriver::onAudioFrame, this);

        actual_channels_ = audio_->initFull(
            audio_device_primary_,
            usb_sleep_reset_,
            topic_audio_main_,
            channel_topics_,
            frame_fn,
            reset_fn);

        if (actual_channels_ == 0)
        {
            RCLCPP_ERROR(get_logger(), "Audio init failed");
            audio_.reset();
            return false;
        }

        return true;
    }

    bool usbResetCallback()
    {
        if (!usb_)
            return false;
        return usb_->resetDevice();
    }

    // ================================================================
    //  ROS2 interfaces
    // ================================================================

    void createRosInterfaces()
    {
        // Publishers
        pub_doa_ = this->create_publisher<std_msgs::msg::Int32>(topic_doa_, 10);

        // Services
        srv_set_mode_ = this->create_service<robohead_interfaces::srv::SimpleCommand>(
            srv_name_mode_,
            std::bind(&RespeakerDriver::onSetMode, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_set_brightness_ = this->create_service<robohead_interfaces::srv::SimpleCommand>(
            srv_name_brightness_,
            std::bind(&RespeakerDriver::onSetBrightness, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_set_color_all_ = this->create_service<robohead_interfaces::srv::Color>(
            srv_name_color_all_,
            std::bind(&RespeakerDriver::onSetColorAll, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_set_color_palette_ = this->create_service<robohead_interfaces::srv::ColorPalette>(
            srv_name_palette_,
            std::bind(&RespeakerDriver::onSetColorPalette, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Subscriptions
        sub_set_color_manual_ = this->create_subscription<robohead_interfaces::msg::ColorArray>(
            topic_color_manual_, 10,
            std::bind(&RespeakerDriver::onSetColorManual, this,
                      std::placeholders::_1));
    }

    // ================================================================
    //  Audio frame callback (DOA + speech)
    // ================================================================

    void onAudioFrame()
    {
        if (!usb_)
            return;

        if (!usb_->isOpen())
            return;

        // DOA
        int32_t raw_doa = usb_->readDoaAngle();

        double rad = raw_doa * M_PI / 180.0;
        double cos_off = std::cos(doa_yaw_offset_rad_);
        double sin_off = std::sin(doa_yaw_offset_rad_);
        double x = std::cos(rad);
        double y = std::sin(rad);

        double x_corrected =  x * cos_off + y * sin_off;
        double y_corrected = -x * sin_off + y * cos_off;

        double corrected_rad = std::atan2(y_corrected, x_corrected);
        int corrected_deg = static_cast<int>(
            std::round(corrected_rad * 180.0 / M_PI));

        std_msgs::msg::Int32 doa_msg;
        doa_msg.data = corrected_deg;
        pub_doa_->publish(doa_msg);

    }

    // ================================================================
    //  Service callbacks
    // ================================================================

    void onSetMode(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        if (usb_ && usb_->ledSetMode(request->data))
            response->data = 0;
        else
            response->data = -1;
    }

    void onSetBrightness(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        uint8_t brightness = static_cast<uint8_t>(request->data);

        if (usb_ && usb_->ledSetBrightness(brightness))
            response->data = 0;
        else
            response->data = -1;
    }

    void onSetColorAll(
        const std::shared_ptr<robohead_interfaces::srv::Color::Request> request,
        std::shared_ptr<robohead_interfaces::srv::Color::Response> response)
    {
        if (usb_ && usb_->ledSetColorAll(request->red, request->green, request->blue))
            response->data = 0;
        else
            response->data = -1;
    }

    void onSetColorPalette(
        const std::shared_ptr<robohead_interfaces::srv::ColorPalette::Request> request,
        std::shared_ptr<robohead_interfaces::srv::ColorPalette::Response> response)
    {
        UsbHandler::LedColor color_a(
            request->color_a.red,
            request->color_a.green,
            request->color_a.blue);

        UsbHandler::LedColor color_b(
            request->color_b.red,
            request->color_b.green,
            request->color_b.blue);

        if (usb_ && usb_->ledSetColorPalette(color_a, color_b))
            response->data = 0;
        else
            response->data = -1;
    }

    void onSetColorManual(
        const robohead_interfaces::msg::ColorArray::SharedPtr msg)
    {
        if (!usb_)
            return;

        int expected = UsbHandler::NUM_LEDS;
        int received = static_cast<int>(msg->colors.size());

        if (received != expected)
        {
            RCLCPP_WARN(get_logger(),
                        "Expected %d colors, got %d", expected, received);
            return;
        }

        std::vector<UsbHandler::LedColor> colors(expected);
        for (int i = 0; i < expected; ++i)
        {
            colors[i].r = msg->colors[i].red;
            colors[i].g = msg->colors[i].green;
            colors[i].b = msg->colors[i].blue;
        }

        usb_->ledSetColorManual(colors);
    }

    // ================================================================
    //  Members
    // ================================================================

    // Handlers
    std::unique_ptr<UsbHandler>   usb_;
    std::unique_ptr<AudioHandler> audio_;

    // Auto-detected
    std::string detected_version_;
    int         actual_channels_;

    // Params
    double   doa_yaw_offset_rad_;
    uint16_t usb_vid_;
    uint16_t usb_pid_;
    int      usb_timeout_;
    int      usb_sleep_reset_;
    int      usb_sleep_stop_;

    AudioHandler::Config audio_cfg_;
    std::string audio_device_primary_;
    std::string audio_device_fallback_;

    // Topic / service names
    std::string topic_audio_main_;
    std::vector<std::string> channel_topics_;
    std::string topic_doa_;
    std::string topic_speech_;
    std::string topic_color_manual_;
    std::string srv_name_mode_;
    std::string srv_name_brightness_;
    std::string srv_name_color_all_;
    std::string srv_name_palette_;

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_doa_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  pub_speech_;

    // ROS services
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_set_mode_;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_set_brightness_;
    rclcpp::Service<robohead_interfaces::srv::Color>::SharedPtr         srv_set_color_all_;
    rclcpp::Service<robohead_interfaces::srv::ColorPalette>::SharedPtr  srv_set_color_palette_;

    // ROS subscriptions
    rclcpp::Subscription<robohead_interfaces::msg::ColorArray>::SharedPtr sub_set_color_manual_;
};

// ================================================================
//  ALSA error suppression
// ================================================================

static void alsa_error_handler(const char * /*file*/, int /*line*/,
                               const char * /*function*/, int /*err*/,
                               const char * /*fmt*/, ...)
{
}

// ================================================================
//  main
// ================================================================

int main(int argc, char *argv[])
{
    snd_lib_error_set_handler(alsa_error_handler);

    rclcpp::init(argc, argv);

    std::shared_ptr<RespeakerDriver> node =
        std::make_shared<RespeakerDriver>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}