#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string>
#include <nlohmann/json.hpp>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <curl/curl.h>
#include "robohead_interfaces/srv/simple_command.hpp"
#include "robohead_interfaces/srv/play_media.hpp"

class MediaDriver : public rclcpp::Node
{
public:
    MediaDriver() : Node("media_driver"), stop_thread_(false)
    {
        this->declare_parameter("mpv_socket", "/tmp/mpv_display.sock");
        this->declare_parameter("screen_rotate", 3);
        this->declare_parameter("srv_play_media_name", "~/play_media");
        this->declare_parameter("srv_set_volume_name", "~/set_volume");
        this->declare_parameter("srv_get_volume_name", "~/get_volume");

        this->get_parameter("mpv_socket", mpv_socket_);
        this->get_parameter("screen_rotate", screen_rotate_);
        this->get_parameter("srv_play_media_name", srv_play_media_name);
        this->get_parameter("srv_set_volume_name", srv_set_volume_name);
        this->get_parameter("srv_get_volume_name", srv_get_volume_name);

        srv_play_media = this->create_service<robohead_interfaces::srv::PlayMedia>(
            srv_play_media_name,
            std::bind(&MediaDriver::playMediaCallback, this, std::placeholders::_1, std::placeholders::_2));

        srv_get_volume = this->create_service<robohead_interfaces::srv::SimpleCommand>(
            srv_get_volume_name,
            std::bind(&MediaDriver::getVolumeCallback, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_volume = this->create_service<robohead_interfaces::srv::SimpleCommand>(
            srv_set_volume_name,
            std::bind(&MediaDriver::setVolumeCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 🔧 Запускаем поток отправки команд
        sender_thread_ = std::thread(&MediaDriver::senderLoop, this);

        RCLCPP_INFO(this->get_logger(), "INITED: media_driver. Persistent socket: %s", mpv_socket_.c_str());
    }

    ~MediaDriver()
    {
        stop_thread_ = true;
        cv_.notify_all();
        if (sender_thread_.joinable())
            sender_thread_.join();
        if (sock_fd_ >= 0)
            close(sock_fd_);
    }

private:
    std::string mpv_socket_;
    std::string srv_play_media_name;
    std::string srv_set_volume_name;
    std::string srv_get_volume_name;
    int screen_rotate_ = 0;

    rclcpp::Service<robohead_interfaces::srv::PlayMedia>::SharedPtr srv_play_media;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_get_volume;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_set_volume;

    int sock_fd_ = -1;
    std::thread sender_thread_;
    std::queue<nlohmann::json> command_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool stop_thread_;

    // ---------- NETWORK SETUP ----------

    bool ensureConnected()
    {
        if (sock_fd_ >= 0)
            return true;

        sock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock_fd_ < 0)
            return false;

        struct sockaddr_un addr = {};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mpv_socket_.c_str(), sizeof(addr.sun_path) - 1);

        if (connect(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to MPV socket");
        return true;
    }

    // ---------- COMMAND SENDER THREAD ----------

// echo '{"command":["loadfile","/home/pirobohead_ws/src/robohead2/robohead_controller/actions/std_greeting/greeting.png","replace"]}' | socat - /tmp/mpv_display.sock
    void senderLoop()
    {
        while (!stop_thread_)
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [&]
                     { return !command_queue_.empty() || stop_thread_; });
            if (stop_thread_)
                break;

            auto cmd = command_queue_.front();
            command_queue_.pop();
            lock.unlock();

            if (!ensureConnected())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            std::string msg = cmd.dump() + "\n";
            ssize_t sent = send(sock_fd_, msg.c_str(), msg.size(), 0);
            if (sent < 0)
            {
                RCLCPP_WARN(this->get_logger(), "Lost MPV connection, will reconnect");
                close(sock_fd_);
                sock_fd_ = -1;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void queueCommand(const nlohmann::json &cmd)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(cmd);
        cv_.notify_one();
    }

    // ---------- UTILS ----------

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
    {
        (void)contents;
        (void)userp;
        return size * nmemb;
    }

    bool is_url_accessible(const std::string &url, int timeout_sec = 5)
    {
        CURL *curl = curl_easy_init();
        if (!curl)
            return false;

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        CURLcode res = curl_easy_perform(curl);
        bool ok = false;
        if (res == CURLE_OK)
        {
            long code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
            ok = (code == 200 || code == 206);
        }
        curl_easy_cleanup(curl);
        return ok;
    }

    // ---------- SERVICE CALLBACKS ----------

    void playMediaCallback(
        const std::shared_ptr<robohead_interfaces::srv::PlayMedia::Request> request,
        std::shared_ptr<robohead_interfaces::srv::PlayMedia::Response> response)
    {
        response->data = -1;

        auto is_url = [](const std::string &s)
        { return s.rfind("http://", 0) == 0 || s.rfind("https://", 0) == 0; };

        if (request->path_to_media_file.empty())
        {
            queueCommand({{"command", {"stop"}}});
            response->data = 0;
            return;
        }

        // Проверяем наличие файла/доступность
        if (is_url(request->path_to_media_file))
        {
            if (!is_url_accessible(request->path_to_media_file))
            {
                RCLCPP_ERROR(this->get_logger(), "URL not accessible: %s", request->path_to_media_file.c_str());
                response->data = -2;
                return;
            }
        }
        else if (!std::filesystem::exists(request->path_to_media_file))
        {
            RCLCPP_ERROR(this->get_logger(), "Media not found: %s", request->path_to_media_file.c_str());
            response->data = -1;
            return;
        }

        // === 1. loadfile ===
        // queueCommand({{"command", {"loadfile", request->path_to_media_file, "replace"}}});
        // queueCommand({{"command", {"set_property", "stream-open-filename", request->path_to_media_file}}});
        // queueCommand({{"command", {"stop"}}});
        // queueCommand({{"command", {"video-clear"}}});
        // queueCommand({{"command", {"loadfile", request->path_to_media_file, "replace"}}});
        // std::string file_with_dummy = request->path_to_media_file + "?t=" + std::to_string(time(nullptr));

        // Тут костыль!!
        queueCommand({{"command", {"loadfile", request->path_to_media_file, "replace"}}});
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        queueCommand({{"command", {"loadfile", request->path_to_media_file, "replace"}}});
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));


        // queueCommand({{"command", {"loadfile", request->path_to_media_file, "replace"}}});

        // queueCommand({{"command", {"loadfile", request->path_to_media_file, "replace"}}});

    


        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // === 2. rotation ===
        if (screen_rotate_ != 0)
        {
            queueCommand({{"command", {"set_property", "video-rotate", screen_rotate_ * 90}}});
        }

        // === 3. override audio ===
        if (!request->path_to_override_audio_file.empty())
        {
            queueCommand({{"command", {"set_property", "aid", "no"}}});
            queueCommand({{"command", {"audio-add", request->path_to_override_audio_file}}});
            queueCommand({{"command", {"set_property", "aid", "auto"}}});
        }

        // === 4. loop ===
        std::string loop_val = request->is_cycle ? "inf" : "no";
        queueCommand({{"command", {"set_property", "loop-file", loop_val}}});

        // === 5. unpause ===
        queueCommand({{"command", {"set_property", "pause", "no"}}});

        response->data = 0;
        RCLCPP_INFO(this->get_logger(), "Media queued for playback: %s", request->path_to_media_file.c_str());
    }

    void setVolumeCallback(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        int vol = std::max(0, std::min(100, static_cast<int>(request->data)));
        queueCommand({{"command", {"set_property", "volume", vol}}});
        response->data = vol;
    }

    void getVolumeCallback(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        (void)request;
        // В упрощённом варианте можно не опрашивать MPV
        response->data = -1;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MediaDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
