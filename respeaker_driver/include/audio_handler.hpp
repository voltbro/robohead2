#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robohead_interfaces/msg/audio_data.hpp>

#define PA_NO_JACK
#include <portaudio.h>

#include <vector>
#include <string>
#include <functional>
#include <cstdint>

class AudioHandler
{
public:
    static const int MAX_CHANNELS = 6;

    struct Config
    {
        int sample_rate;
        int frames_per_buffer;
        int count_of_channels; // 0 = авто
        int main_channel;

        Config()
            : sample_rate(16000), frames_per_buffer(1024), count_of_channels(0), main_channel(0)
        {
        }
    };

    typedef std::function<void()> FrameCallback;
    typedef std::function<bool()> ResetCallback;

    AudioHandler(rclcpp::Node *node, const Config &config);
    ~AudioHandler();

    /// Полная инициализация. Возвращает реальное число каналов, 0 при ошибке.
    int initFull(const std::string &device_name_primary,
                 int usb_sleep_reset_ms,
                 const std::string &main_topic,
                 const std::vector<std::string> &channel_topics,
                 FrameCallback frame_callback,
                 ResetCallback usb_reset_fn);

    // Пошаговые методы
    bool initPortAudio();
    void terminatePortAudio();
    bool findDevice(const std::string &device_name);
    void createPublishers(const std::string &main_topic,
                          const std::vector<std::string> &channel_topics);
    bool openStream();
    void closeStream();
    void setFrameCallback(FrameCallback cb);

    // Getters
    int getNumChannels() const;
    int getDeviceIndex() const;
    bool isDeviceFound() const;
    bool isPortAudioInitialized() const;

private:
    static std::string toLower(const std::string &str);

    static int paCallback(
        const void *input, void *output,
        unsigned long frameCount,
        const PaStreamCallbackTimeInfo *timeInfo,
        PaStreamCallbackFlags statusFlags,
        void *userData);

    rclcpp::Node *node_;
    Config config_;

    PaStream *stream_;
    int device_index_;
    int num_channels_;
    bool pa_initialized_;

    rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr pub_main_;
    std::vector<rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr> pub_channels_;

    FrameCallback frame_callback_;
};