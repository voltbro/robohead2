#include "audio_handler.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <algorithm>

// ================================================================
//  Constructor / Destructor
// ================================================================

AudioHandler::AudioHandler(rclcpp::Node *node, const Config &config)
    : node_(node)
    , config_(config)
    , stream_(nullptr)
    , device_index_(-1)
    , num_channels_(config.count_of_channels)
    , pa_initialized_(false)
{
}

AudioHandler::~AudioHandler()
{
    closeStream();
    terminatePortAudio();
}

// ================================================================
//  Getters
// ================================================================

int  AudioHandler::getNumChannels()         const { return num_channels_; }
int  AudioHandler::getDeviceIndex()         const { return device_index_; }
bool AudioHandler::isDeviceFound()          const { return device_index_ != -1; }
bool AudioHandler::isPortAudioInitialized() const { return pa_initialized_; }

void AudioHandler::setFrameCallback(FrameCallback cb)
{
    frame_callback_ = cb;
}

// ================================================================
//  Full init
// ================================================================

int AudioHandler::initFull(const std::string &device_name_primary,
                            int usb_sleep_reset_ms,
                            const std::string &main_topic,
                            const std::vector<std::string> &channel_topics,
                            FrameCallback frame_callback,
                            ResetCallback usb_reset_fn)
{
    // 1. Init PortAudio
    if (!initPortAudio())
        return 0;

    // 2. Поиск устройства
    bool found = findDevice(device_name_primary);

    // 3. Не нашли — USB reset + повтор
    if (!found && usb_reset_fn)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Audio device '%s' not found. Trying USB reset...",
                    device_name_primary.c_str());

        if (usb_reset_fn())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(usb_sleep_reset_ms));

            terminatePortAudio();

            if (!initPortAudio())
                return 0;

            found = findDevice(device_name_primary);
        }
    }

    if (!found)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Audio device not found! Check: arecord -l");
        return 0;
    }

    // 4. Определение числа каналов
    int hw_channels = num_channels_;
    if (hw_channels > MAX_CHANNELS)
        hw_channels = MAX_CHANNELS;

    if (config_.count_of_channels > 0)
    {
        if (config_.count_of_channels <= hw_channels)
        {
            num_channels_ = config_.count_of_channels;
            RCLCPP_INFO(node_->get_logger(),
                        "Using %d of %d available channels (configured)",
                        num_channels_, hw_channels);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Requested %d channels but device has %d. Using %d.",
                        config_.count_of_channels, hw_channels, hw_channels);
            num_channels_ = hw_channels;
        }
    }
    else
    {
        num_channels_ = hw_channels;
        RCLCPP_INFO(node_->get_logger(),
                    "Auto-detected %d audio channels", num_channels_);
    }

    // 5. Валидация main_channel
    if (config_.main_channel < 0 || config_.main_channel >= num_channels_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Main channel %d out of range [0..%d), resetting to 0",
                    config_.main_channel, num_channels_);
        config_.main_channel = 0;
    }

    // 6. Паблишеры
    int topic_count = static_cast<int>(channel_topics.size());
    if (topic_count > num_channels_)
        topic_count = num_channels_;

    std::vector<std::string> topics(
        channel_topics.begin(),
        channel_topics.begin() + topic_count);

    createPublishers(main_topic, topics);

    // 7. Frame callback
    if (frame_callback)
        setFrameCallback(frame_callback);

    // 8. Открытие потока
    if (!openStream())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open audio stream");
        return 0;
    }

    return num_channels_;
}

// ================================================================
//  PortAudio: init / terminate
// ================================================================

bool AudioHandler::initPortAudio()
{
    // Подавляем ALSA-сообщения в stderr
    int saved_stderr = dup(STDERR_FILENO);
    int null_fd = open("/dev/null", O_WRONLY);
    bool redirected = (saved_stderr >= 0 && null_fd >= 0);

    if (redirected)
    {
        dup2(null_fd, STDERR_FILENO);
        close(null_fd);
    }

    PaError err = Pa_Initialize();

    if (redirected)
    {
        dup2(saved_stderr, STDERR_FILENO);
        close(saved_stderr);
    }

    if (err != paNoError)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "PortAudio init failed: %s", Pa_GetErrorText(err));
        return false;
    }

    pa_initialized_ = true;
    return true;
}

void AudioHandler::terminatePortAudio()
{
    if (pa_initialized_)
    {
        Pa_Terminate();
        pa_initialized_ = false;
    }
}

// ================================================================
//  Поиск устройства
// ================================================================

std::string AudioHandler::toLower(const std::string &str)
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return result;
}

bool AudioHandler::findDevice(const std::string &device_name)
{
    if (!pa_initialized_)
        return false;

    device_index_ = -1;

    std::string search_name = toLower(device_name);

    int numDevices = Pa_GetDeviceCount();
    for (int i = 0; i < numDevices; i++)
    {
        const PaDeviceInfo *info = Pa_GetDeviceInfo(i);

        if (info == nullptr)
            continue;
        if (info->name == nullptr)
            continue;

        std::string found_name = toLower(std::string(info->name));

        if (found_name.find(search_name) != std::string::npos)
        {
            device_index_ = i;
            num_channels_ = info->maxInputChannels;
            RCLCPP_INFO(node_->get_logger(),
                        "Found audio device: %s (%d channels)",
                        info->name, num_channels_);
            return true;
        }
    }

    return false;
}

// ================================================================
//  Publishers
// ================================================================

void AudioHandler::createPublishers(
    const std::string &main_topic,
    const std::vector<std::string> &channel_topics)
{
    pub_main_ = node_->create_publisher<robohead_interfaces::msg::AudioData>(
        main_topic, 10);

    pub_channels_.clear();
    for (size_t i = 0; i < channel_topics.size(); ++i)
    {
        rclcpp::Publisher<robohead_interfaces::msg::AudioData>::SharedPtr pub =
            node_->create_publisher<robohead_interfaces::msg::AudioData>(
                channel_topics[i], 10);
        pub_channels_.push_back(pub);
    }
}

// ================================================================
//  Stream open / close
// ================================================================

bool AudioHandler::openStream()
{
    if (device_index_ == -1)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "No audio device selected, cannot open stream");
        return false;
    }

    const PaDeviceInfo *device_info = Pa_GetDeviceInfo(device_index_);

    PaStreamParameters inputParams;
    inputParams.device = device_index_;
    inputParams.channelCount = num_channels_;
    inputParams.sampleFormat = paInt16;
    inputParams.suggestedLatency = device_info->defaultLowInputLatency;
    inputParams.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(
        &stream_, // Указатель на поток
        &inputParams, // конфиг микрофона
        nullptr, // конфиг динамиков - nullptr т.к. нет играем звук
        config_.sample_rate,
        config_.frames_per_buffer,
        paClipOff, // флаги для записи
        paCallback, // колбек, когда заполнился буфер frames_per_buffer
        this); // указатель, передающийся в колбек

    if (err != paNoError)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Open stream failed: %s", Pa_GetErrorText(err));
        return false;
    }

    err = Pa_StartStream(stream_);
    if (err != paNoError)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Start stream failed: %s", Pa_GetErrorText(err));
        Pa_CloseStream(stream_);
        stream_ = nullptr;
        return false;
    }

    return true;
}

void AudioHandler::closeStream()
{
    if (stream_ != nullptr)
    {
        Pa_AbortStream(stream_);
        Pa_CloseStream(stream_);
        stream_ = nullptr;
    }
}

// ================================================================
//  PortAudio callback
// ================================================================

int AudioHandler::paCallback(
    const void *input, void * /*output*/,
    unsigned long frameCount,
    const PaStreamCallbackTimeInfo * /*timeInfo*/,
    PaStreamCallbackFlags /*statusFlags*/,
    void *userData)
{
    AudioHandler *self = static_cast<AudioHandler *>(userData);
    const int16_t *data = static_cast<const int16_t *>(input);
    int num_ch = self->num_channels_;
    int pub_count = static_cast<int>(self->pub_channels_.size());

    // Публикация каждого канала
    for (int ch = 0; ch < num_ch && ch < pub_count; ++ch)
    {
        robohead_interfaces::msg::AudioData msg;
        msg.data.resize(frameCount);

        for (unsigned long i = 0; i < frameCount; ++i)
        {
            msg.data[i] = data[i * num_ch + ch];
        }

        self->pub_channels_[ch]->publish(msg);
    }

    // Основной канал
    int main_ch = self->config_.main_channel;
    if (main_ch >= 0 && main_ch < num_ch && self->pub_main_)
    {
        robohead_interfaces::msg::AudioData main_msg;
        main_msg.data.resize(frameCount);

        for (unsigned long i = 0; i < frameCount; ++i)
        {
            main_msg.data[i] = data[i * num_ch + main_ch];
        }

        self->pub_main_->publish(main_msg);
    }

    // Внешний коллбэк
    if (self->frame_callback_)
    {
        self->frame_callback_();
    }

    return paContinue;
}