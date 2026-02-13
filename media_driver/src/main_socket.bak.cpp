#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string>
#include <nlohmann/json.hpp>
#include <thread> // для sleep
#include <poll.h>
#include <curl/curl.h>
#include "robohead_interfaces/srv/simple_command.hpp"                                       
#include "robohead_interfaces/srv/play_media.hpp"   


/* example usage
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_media_file: '/home/pi/loadingVideo.mp4'
path_to_override_audio_file: '/home/pi/file.mp3'
is_block: 0
is_cycle: 0" 
*/

/* example usage
ros2 service call /robohead_controller/media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_media_file: '/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_attention/attention.png'
path_to_override_audio_file: '/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_attention/attention.mp3'
is_block: 0
is_cycle: 0" 
*/

/* example usage
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_media_file: ''
path_to_override_audio_file: ''
is_block: 0
is_cycle: 0" 
*/

/* example usage
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_media_file: 'http://chanson.hostingradio.ru:8041/chanson256.mp3'
path_to_override_audio_file: ''
is_block: 0
is_cycle: 0" 
*/

/* example usage
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_media_file: '/home/pi/video_audio.mov'
path_to_override_audio_file: 'http://chanson.hostingradio.ru:8041/chanson256.mp3'
is_block: 0
is_cycle: 0" 
*/

/* example usage
ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_media_file: '/home/pi/video_audio.mov'
path_to_override_audio_file: ''
is_block: 0
is_cycle: 0" 
*/

/*
ros2 service call /media_driver/set_volume robohead_interfaces/srv/SimpleCommand "data: 30"
*/

/*
ros2 service call /media_driver/get_volume robohead_interfaces/srv/SimpleCommand "data: 0"
*/

class MediaDriver : public rclcpp::Node
{
public:
    MediaDriver() : Node("media_driver")
    {
        this->declare_parameter("mpv_socket", "/tmp/mpv_display.sock");
        this->declare_parameter("screen_rotate", 3); // 0=0°, 1=90°, 2=180°, 3=270°
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
                                                                            std::bind(&MediaDriver::playMediaCallback, this,
                                                                            std::placeholders::_1, std::placeholders::_2)
                                                                            );

        srv_get_volume = this->create_service<robohead_interfaces::srv::SimpleCommand>(
                                                                                srv_get_volume_name,
                                                                                std::bind(&MediaDriver::getVolumeCallback, this,
                                                                                std::placeholders::_1, std::placeholders::_2)
                                                                                );

        srv_set_volume = this->create_service<robohead_interfaces::srv::SimpleCommand>(
                                                                                srv_set_volume_name,
                                                                                std::bind(&MediaDriver::setVolumeCallback, this,
                                                                                std::placeholders::_1, std::placeholders::_2)
                                                                                );

        RCLCPP_INFO(this->get_logger(), "INITED: media_driver. Socket: %s", mpv_socket_.c_str());
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


    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        (void)contents;
        (void)userp;
        return size * nmemb; // Принимаем данные, но не сохраняем
    }

    bool is_url_accessible(const std::string& url, int timeout_sec = 5) {
        CURL *curl;
        CURLcode res;
        bool accessible = false;

        curl = curl_easy_init();
        if (!curl) return false;

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_NOBODY, 1L); // HEAD-запрос
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // следовать редиректам

        res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            long response_code;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            accessible = (response_code == 200 || response_code == 206); // 206 = Partial Content (для потоков)
        }

        curl_easy_cleanup(curl);
        return accessible;
    }


    // Только отправка (без ответа)
    // bool sendCommandNoReply(const nlohmann::json& cmd)
    // {
    //     int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    //     if (sock < 0) return false;

    //     struct sockaddr_un addr = {};
    //     addr.sun_family = AF_UNIX;
    //     strncpy(addr.sun_path, mpv_socket_.c_str(), sizeof(addr.sun_path) - 1);

    //     if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    //         close(sock);
    //         return false;
    //     }

    //     std::string msg = cmd.dump() + "\n";
    //     send(sock, msg.c_str(), msg.size(), 0);
    //     close(sock);
    //     return true;
    // }

    bool sendCommandNoReply(const nlohmann::json& cmd)
    {
        int sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock < 0) return false;

        struct sockaddr_un addr = {};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mpv_socket_.c_str(), sizeof(addr.sun_path) - 1);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock);
            return false;
        }

        std::string msg = cmd.dump() + "\n";
        ssize_t sent = send(sock, msg.c_str(), msg.size(), 0);
        if (sent < 0) {
            close(sock);
            return false;
        }

        // 💡 Дать mpv время принять данные
        shutdown(sock, SHUT_WR);

        // Ждём подтверждение от MPV (даже если не читаем его)
        struct pollfd pfd = {sock, POLLIN, 0};
        poll(&pfd, 1, 100); // 100 мс, чтобы mpv обработал команду

        close(sock);
        return true;
    }


    // Отправка + получение ответа
    std::string sendCommandWithReply(const nlohmann::json& cmd)
    {
        int sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock < 0) return "";

        struct sockaddr_un addr = {};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, mpv_socket_.c_str(), sizeof(addr.sun_path) - 1);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock);
            return "";
        }

        std::string msg = cmd.dump() + "\n";
        if (send(sock, msg.c_str(), msg.size(), 0) <= 0) {
            close(sock);
            return "";
        }

        // Ждём ответ с таймаутом
        struct pollfd pfd = {sock, POLLIN, 0};
        if (poll(&pfd, 1, 2000) <= 0) { // 2 сек
            close(sock);
            RCLCPP_ERROR(this->get_logger(), "sendCommandWithReply: timeout");
            return "";
        }

        char buffer[4096];
        ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
        close(sock);

        if (n <= 0) return "";
        buffer[n] = '\0';
        return std::string(buffer);
    }

    void playMediaCallback(
        const std::shared_ptr<robohead_interfaces::srv::PlayMedia::Request> request,
        std::shared_ptr<robohead_interfaces::srv::PlayMedia::Response> response)
    {
        response->data = -1;


    if (request->path_to_media_file.empty()) {
        // Правильный формат команды
        sendCommandNoReply(nlohmann::json::object({
            {"command", nlohmann::json::array({"stop"})}
        }));
        response->data = 0;
        return;
    }


    auto is_url = [](const std::string& s) -> bool {
        return s.find("http://") == 0 || s.find("https://") == 0 || s.find("rtmp://") == 0;
    };

    if (is_url(request->path_to_media_file)) {
        RCLCPP_INFO(this->get_logger(), "Detected URL: %s", request->path_to_media_file.c_str());
        if (!is_url_accessible(request->path_to_media_file)) {
            RCLCPP_ERROR(this->get_logger(), "URL is not accessible: %s", request->path_to_media_file.c_str());
            response->data = -2; // ошибка: недоступен URL
            return;
        }
    } else {
        // Локальный файл
        if (!std::filesystem::exists(request->path_to_media_file)) {
            RCLCPP_ERROR(this->get_logger(), "Media file not found: %s", request->path_to_media_file.c_str());
            response->data = -1;
            return;
        }
    }

    if (!request->path_to_override_audio_file.empty())
    {
    if (is_url(request->path_to_override_audio_file)) {
        RCLCPP_INFO(this->get_logger(), "Detected URL: %s", request->path_to_override_audio_file.c_str());
        if (!is_url_accessible(request->path_to_override_audio_file)) {
            RCLCPP_ERROR(this->get_logger(), "URL is not accessible: %s", request->path_to_override_audio_file.c_str());
            response->data = -2; // ошибка: недоступен URL
            return;
        }
    } else {
        // Локальный файл
        if (!std::filesystem::exists(request->path_to_override_audio_file)) {
            RCLCPP_ERROR(this->get_logger(), "Media file not found: %s", request->path_to_override_audio_file.c_str());
            response->data = -1;
            return;
        }
    }
    }

    // 1. Загрузка файла
    sendCommandNoReply(nlohmann::json::object({
        {"command", nlohmann::json::array({
            "loadfile", request->path_to_media_file, "replace"
        })}
    }));

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // 2. Поворот
    if (screen_rotate_ != 0) {
        sendCommandNoReply(nlohmann::json::object({
            {"command", nlohmann::json::array({
                "set_property", "video-rotate", screen_rotate_ * 90
            })}
        }));
    }

    // 3. Внешнее аудио
    if (!request->path_to_override_audio_file.empty()) {
        sendCommandNoReply(nlohmann::json::object({
            {"command", nlohmann::json::array({"set_property", "aid", "no"})}
        }));

        sendCommandNoReply(nlohmann::json::object({
            {"command", nlohmann::json::array({"audio-add", request->path_to_override_audio_file})}
        }));

        sendCommandNoReply(nlohmann::json::object({
        {"command", nlohmann::json::array({"set_property", "aid", "auto"})}
        }));
    }



    // 4. Цикл
    std::string loop_val = request->is_cycle ? "inf" : "no";
    sendCommandNoReply(nlohmann::json::object({
        {"command", nlohmann::json::array({
            "set_property", "loop-file", loop_val
        })}
    }));

    // 5. Пауза
    sendCommandNoReply(nlohmann::json::object({
        {"command", nlohmann::json::array({
            "set_property", "pause", "no"
        })}
    }));

    response->data = 0;
    RCLCPP_INFO(this->get_logger(), "Media driver play: %s", request->path_to_media_file.c_str());
    

    if (request->is_block) {
        RCLCPP_INFO(this->get_logger(), "Blocking mode: waiting for playback to finish...");

        while (rclcpp::ok()) {
            // Проверяем, завершилось ли воспроизведение
            auto cmd = nlohmann::json::object({
                {"command", nlohmann::json::array({"get_property", "eof-reached"})}
            });

            std::string reply = sendCommandWithReply(cmd);
            bool eof_reached = false;

            if (!reply.empty()) {
                try {
                    auto j = nlohmann::json::parse(reply);
                    if (j.contains("data") && j["data"].is_boolean()) {
                        eof_reached = j["data"].get<bool>();
                    }
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse eof-reached");
                }
            }

            if (eof_reached) {
                RCLCPP_INFO(this->get_logger(), "Playback finished");
                break;
            }

            // Проверяем, не остановлен ли узел
            if (!rclcpp::ok()) break;

            // Ждём 100 мс перед следующей проверкой
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

}


    void setVolumeCallback(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        // Ограничиваем значение 0-100
        int vol = std::max(0, std::min(100, static_cast<int>(request->data)));

        sendCommandNoReply(nlohmann::json::object({
            {"command", nlohmann::json::array({"set_property", "volume", vol})}
        }));

        response->data = static_cast<int16_t>(vol);
    }


        // Получение громкости
    void getVolumeCallback(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        (void)request;

        auto cmd = nlohmann::json::object({
            {"command", nlohmann::json::array({"get_property", "volume"})}
        });

        std::string reply = sendCommandWithReply(cmd);
        if (reply.empty()) {
            response->data = -1;
            return;
        }

        try {
            auto j = nlohmann::json::parse(reply);
            if (j.contains("data") && j["data"].is_number()) {
                int vol = j["data"].get<int>();
                vol = std::max(0, std::min(100, vol));
                response->data = static_cast<int16_t>(vol);
            } else {
                response->data = -1;
            }
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse volume response: %s", reply.c_str());
            response->data = -1;
        }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<MediaDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}