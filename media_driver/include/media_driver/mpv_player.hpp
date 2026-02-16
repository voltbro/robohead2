#pragma once

#include <mpv/client.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <memory>
#include <filesystem>

namespace MPV
{
  enum class Type
  {
    VIDEO, // Вывод на экран (vo=drm, no-audio=yes)
    AUDIO  // Только звук (vo=null)
  };

  struct Config
  {
    Type type = Type::VIDEO;
    std::string name = "mpv";
    double volume = 50.0;            // 0-100
    int event_wait_timeout_ms = 100; // Таймаут ожидания событий mpv
    int command_timeout_ms = 10;     // Пауза после команды stop/load
    std::string rotate = "0";            // 0-359 в градусах
  };

  class MPVPlayer
  {
  public:
    MPVPlayer(rclcpp::Logger logger, const struct MPV::Config config);
    ~MPVPlayer();

    // Инициализация и управление жизненным циклом
    bool initialize();

    // Управление воспроизведением
    bool play(const std::string &path, bool loop = false);
    bool update_frame(const std::string& path);
    bool stop();
    double set_volume(double volume); // 0.0-100.0
    double get_volume() const;

    // Ожидание событий с поддержкой отмены
    bool is_eof();

  private:
    // Внутренние методы
    void event_loop();
    int safe_command(const std::vector<const char *> &args);

    bool check_file_exists(const std::string &path) const;

    // Конфигурация плееров
    bool configure_video_player();
    bool configure_audio_player();

    // Данные экземпляра
    rclcpp::Logger logger_;
    mpv_handle *mpv_handle_ = nullptr;
    Config config_;
    mutable std::mutex mutex_;
    std::thread event_thread_;
    std::atomic<bool> running_{false};
  };
}
