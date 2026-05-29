#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "robohead_interfaces/srv/play_media.hpp"
#include "robohead_interfaces/srv/simple_command.hpp"
#include "media_driver/mpv_player.hpp"
#include "media_driver/utils.hpp"
#include "media_driver/node.hpp"

#include "robohead_interfaces/msg/audio_data.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std::chrono_literals;
using PlayMedia = robohead_interfaces::srv::PlayMedia;
using SimpleCommand = robohead_interfaces::srv::SimpleCommand;


MediaDriverNode::MediaDriverNode()
        : Node("media_driver"), running_(true)
    {
        // Создаём и инициализируем видео-плеер
        std::string display_rotate = this->declare_parameter<std::string>("display_rotate", "0");
        stop_command = this->declare_parameter<std::string>("stop_command", "__STOP__");
        double volume = this->declare_parameter<double>("default_volume", 50.0);

        std::string srv_set_volume_name = this->declare_parameter<std::string>("srv_set_volume_name", "set_volume");
        std::string srv_get_volume_name = this->declare_parameter<std::string>("srv_get_volume_name", "get_volume");
        std::string srv_play_media_name = this->declare_parameter<std::string>("srv_play_media_name", "play_media");

        std::string srv_is_idle_audio_name = this->declare_parameter<std::string>("srv_is_idle_audio_name", "is_idle/audio");
        std::string srv_is_idle_display_name = this->declare_parameter<std::string>("srv_is_idle_dispaly_name", "is_idle/display");

        std::string topic_stream_name = this->declare_parameter<std::string>("topic_stream_name", "stream");

        MPV::Config video_cfg;
        video_cfg.type = MPV::Type::VIDEO;
        video_cfg.name = "mpv_video";
        video_cfg.rotate = display_rotate;

        video_player_ = std::make_unique<MPV::MPVPlayer>(this->get_logger(), video_cfg);

        if (!video_player_->initialize())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize video player");
            throw std::runtime_error("Video player init failed");
        }

        MPV::Config audio_cfg;
        audio_cfg.type = MPV::Type::AUDIO;
        audio_cfg.name = "audio_mpv";
        audio_cfg.volume = volume;
        // Создаём и инициализируем аудио-плеер
        audio_player_ = std::make_unique<MPV::MPVPlayer>(this->get_logger(), audio_cfg);

        if (!audio_player_->initialize())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize audio player");
            throw std::runtime_error("Audio player init failed");
        }

        // === Аудиострим ===
        if (mkfifo(audio_fifo_path_.c_str(), 0666) != 0 && errno != EEXIST) {
            RCLCPP_ERROR(this->get_logger(), "FIFO create failed: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "FIFO ready: %s", audio_fifo_path_.c_str());
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Даем MPV время открыть файл
        fifo_fd_ = open(audio_fifo_path_.c_str(), O_RDWR | O_NONBLOCK);
        if (fifo_fd_ < 0) {
            if (errno == ENXIO) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "⏳ FIFO reader not connected yet (ENXIO)");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Open FIFO failed: %s", strerror(errno));
            }
            return;
        }
        RCLCPP_INFO(this->get_logger(), "FIFO writer connected descriptor: %d", fifo_fd_);

        sub_audio_stream_ = this->create_subscription<robohead_interfaces::msg::AudioData>(
            "audio_stream",
            rclcpp::QoS(1).best_effort(),
            [this](const robohead_interfaces::msg::AudioData::SharedPtr msg) { handle_stream_audio(msg); });

        // RCLCPP_INFO(this->get_logger(), "Audio stream ready: %s", audio_fifo_path_.c_str());

        // Сервисы
        srv_play_ = this->create_service<PlayMedia>(
            srv_play_media_name,
            std::bind(&MediaDriverNode::handle_play_media, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_vol_ = this->create_service<SimpleCommand>(
            srv_set_volume_name,
            std::bind(&MediaDriverNode::handle_set_volume, this, std::placeholders::_1, std::placeholders::_2));

        srv_get_vol_ = this->create_service<SimpleCommand>(
            srv_get_volume_name,
            std::bind(&MediaDriverNode::handle_get_volume, this, std::placeholders::_1, std::placeholders::_2));


        srv_is_idle_audio_ = this->create_service<SimpleCommand>(
            srv_is_idle_audio_name,
            std::bind(&MediaDriverNode::handle_is_idle_audio, this, std::placeholders::_1, std::placeholders::_2));
        srv_is_idle_display_ = this->create_service<SimpleCommand>(
            srv_is_idle_display_name,
            std::bind(&MediaDriverNode::handle_is_idle_display, this, std::placeholders::_1, std::placeholders::_2));

        // Подписка на видеопоток
        sub_stream_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_stream_name,
            rclcpp::QoS(1).best_effort(),
            std::bind(&MediaDriverNode::handle_stream_image, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "INITED");
    }

MediaDriverNode::~MediaDriverNode()
{
    running_.store(false);
    close(fifo_fd_);
    // Деструкторы автоматически вызовут shutdown() для обоих плееров
}


void MediaDriverNode::handle_stream_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(mtx_);

    try
    {
        // Конвертируем ROS Image → OpenCV
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Сохраняем кадр в /dev/shm (в памяти, без диска)
        std::string tmp_path = "/dev/shm/robohead_stream_frame.ppm";
        cv::imwrite(tmp_path, frame);

        // Загружаем в видео-плеер БЕЗ остановки аудио!
        video_player_->update_frame(tmp_path);

        RCLCPP_DEBUG(this->get_logger(), "Stream frame updated (%dx%d)", frame.cols, frame.rows);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Stream error: %s", e.what());
    }
}

void MediaDriverNode::handle_play_media(
    const std::shared_ptr<PlayMedia::Request> request,
    std::shared_ptr<PlayMedia::Response> response)
{

    // 🟢 Если активен стрим — глушим его перед запуском файла/потока
    if (audio_stream_active_) {
        std::lock_guard<std::mutex> lk(audio_fifo_mutex_);
        audio_stream_active_ = false;
        audio_player_->stop();
    }

    response->data = -1;
    std::string video_path = request->path_to_video_file;
    std::string audio_path = request->path_to_audio_file;
    bool loop = request->loop;

    RCLCPP_INFO(this->get_logger(),
                "[media_driver] Request: video='%s', audio='%s', loop=%s",
                video_path.empty() ? "none" : video_path.c_str(),
                audio_path.empty() ? "none" : audio_path.c_str(),
                loop ? "yes" : "no");

    // Обработка видео (если указан путь)
    if (!video_path.empty())
    {   

        if (video_path == stop_command)
        {
            video_player_->stop();
        } else if (is_video(video_path) || is_image(video_path) || is_url(audio_path))
        {
            if (!video_player_->play(video_path, loop))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to play video: %s", video_path.c_str());
                response->data = -1;
                return;
            }
        } else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid video/image path: %s", video_path.c_str());
            response->data = -1;
            return;
        }
    }

    // Обработка аудио (если указан путь)
    if (!audio_path.empty())
    {
        if (audio_path == stop_command)
        {
            audio_player_->stop();
        } else if (is_audio(audio_path) || is_video(audio_path) || is_url(audio_path))
        {
            if (!audio_player_->play(audio_path, loop))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to play audio: %s", audio_path.c_str());
                response->data = -1;
                return;
            }
        } else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid audio path: %s", audio_path.c_str());
            response->data = -1;
            return;
        }
    }

    // Успешное завершение
    response->data = 0;
    RCLCPP_INFO(this->get_logger(), "[media_driver] Playback started successfully");
}

void MediaDriverNode::handle_set_volume(
    const std::shared_ptr<SimpleCommand::Request> request,
    std::shared_ptr<SimpleCommand::Response> response)
{
    int vol = request->data;
    vol = std::clamp(vol, 0, 100);

    double res = audio_player_->set_volume(vol);
    RCLCPP_WARN(this->get_logger(), "volume: %f", res);

    response->data = static_cast<int>(res);
}

void MediaDriverNode::handle_get_volume(
    const std::shared_ptr<SimpleCommand::Request> /*request*/,
    std::shared_ptr<SimpleCommand::Response> response)
{
    double vol = audio_player_->get_volume(); // Берём громкость аудио-плеера
    response->data = static_cast<int16_t>(vol);
}

void MediaDriverNode::handle_is_idle_audio(
    const std::shared_ptr<SimpleCommand::Request>,
    std::shared_ptr<SimpleCommand::Response> response)
{
    bool is_idle = audio_player_->is_idle();
    response->data = static_cast<int16_t>(is_idle);
}

void MediaDriverNode::handle_is_idle_display(
    const std::shared_ptr<SimpleCommand::Request>,
    std::shared_ptr<SimpleCommand::Response> response)
{
    bool is_idle = video_player_->is_idle();
    response->data = static_cast<int16_t>(is_idle);
}

void MediaDriverNode::handle_stream_audio(const robohead_interfaces::msg::AudioData::SharedPtr msg)
{
    if (msg->data.empty()) return;
    

    // Сначала запускаем MPV, чтобы он открыл FIFO на чтение
    if (!audio_stream_active_) {
        RCLCPP_ERROR(this->get_logger(), "Open stream...");
        if (audio_player_->play_from_pipe(audio_fifo_path_, 16000, 1)) {
            audio_stream_active_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream");
            return;
        }
    }

    size_t bytes_to_write = msg->data.size() * sizeof(int16_t);
    ssize_t written = write(fifo_fd_, msg->data.data(), bytes_to_write);
    
    if (written > 0) {
        RCLCPP_DEBUG(this->get_logger(), "Wrote %zd/%zu bytes to FIFO", written, bytes_to_write);
    } else if (written < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FIFO buffer full!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "write() failed: %s", strerror(errno));
            // Если пайп сломался (например, MPV перезапустился), сбрасываем FD
            close(fifo_fd_);
            fifo_fd_ = -1;
            audio_stream_active_ = false;
        }
    }
}
