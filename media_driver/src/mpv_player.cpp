#include "media_driver/mpv_player.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <system_error>
#include <sys/stat.h>

using namespace std::chrono_literals;
using namespace MPV;

#include <filesystem>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
/**
 * Ищет аудиокарту, в названии которой содержится указанный паттерн.
 * Возвращает строку формата "alsa/plughw:X,0"
 */
std::string MPV::find_audio_device(const std::string& pattern) {
    namespace fs = std::filesystem;
    
    // Приводим паттерн к нижнему регистру для поиска
    std::string pattern_low = pattern;
    std::transform(pattern_low.begin(), pattern_low.end(), pattern_low.begin(), ::tolower);

    // Перебираем индексы карт от 0 до 7 (обычно их не больше)
    for (int i = 0; i < 8; ++i) {
        std::string card_path = "/proc/asound/card" + std::to_string(i);
        std::string id_file_path = card_path + "/id";

        if (fs::exists(id_file_path)) {
            std::ifstream id_file(id_file_path);
            std::string card_id;
            
            if (id_file >> card_id) {
                // Переводим ID карты в нижний регистр
                std::string card_id_low = card_id;
                std::transform(card_id_low.begin(), card_id_low.end(), card_id_low.begin(), ::tolower);

                // Если паттерн найден в ID
                if (card_id_low.find(pattern_low) != std::string::npos) {
                    // Возвращаем строку с ИНДЕКСОМ карты
                    return "alsa/plughw:" + std::to_string(i) + ",0";
                }
            }
        }
    }

    // Если ничего не нашли, пробуем default
    return "alsa/default";
}



MPVPlayer::MPVPlayer(rclcpp::Logger logger, const struct MPV::Config config) : logger_(logger), config_(config)
{
  RCLCPP_DEBUG(logger_, "[%s] Creating MPVPlayer (type: %s, volume: %f)",
               config_.name.c_str(), config_.type == MPV::Type::VIDEO ? "VIDEO" : "AUDIO", config_.volume);
}

MPVPlayer::~MPVPlayer()
{
  // Атомарная остановка потока событий
  running_.store(false);

  // Остановка воспроизведения
  if (mpv_handle_)
  {
    safe_command({"stop"});
    mpv_terminate_destroy(mpv_handle_);
    mpv_handle_ = nullptr;
    RCLCPP_INFO(logger_, "[%s] MPV terminated", config_.name.c_str());
  }

  // Ожидание завершения потока событий
  if (event_thread_.joinable())
  {
    event_thread_.join();
    RCLCPP_DEBUG(logger_, "[%s] Event thread joined", config_.name.c_str());
  }
}

bool MPVPlayer::initialize()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Создание экземпляра mpv
  mpv_handle_ = mpv_create();
  if (!mpv_handle_)
  {
    RCLCPP_FATAL(logger_, "[%s] mpv_create() FAILED - cannot create mpv instance", config_.name.c_str());
    return false;
  }

  // Специфичные настройки по типу
  bool configured = false;
  if (config_.type == MPV::Type::VIDEO)
  {
    configured = configure_video_player();
  }
  else if (config_.type == MPV::Type::AUDIO)
  {
    configured = configure_audio_player();
  }
  else
  {
    RCLCPP_FATAL(logger_, "[%s] mpv configure FAILED. Invalid type (VIDEO/AUDIO) in config!", config_.name.c_str());
  }

  if (!configured)
  {
    RCLCPP_FATAL(logger_, "[%s] mpv configure total FAILED.", config_.name.c_str());
    mpv_terminate_destroy(mpv_handle_);
    mpv_handle_ = nullptr;
    return false;
  }

  // Установка громкости
  if (mpv_set_property(mpv_handle_, "volume", MPV_FORMAT_DOUBLE, &config_.volume) < 0)
  {
    RCLCPP_WARN(logger_, "[%s] Failed to set initial volume to %f", config_.name.c_str(), config_.volume);
  }

  // Инициализация mpv
  if (mpv_initialize(mpv_handle_) < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] mpv_initialize() FAILED", config_.name.c_str());
    mpv_terminate_destroy(mpv_handle_);
    mpv_handle_ = nullptr;
    return false;
  }

  // Запуск потока событий
  running_ = true;
  event_thread_ = std::thread(&MPVPlayer::event_loop, this);

  RCLCPP_INFO(logger_, "[%s] MPV player initialized successfully (type: %s)",
              config_.name.c_str(), config_.type == MPV::Type::VIDEO ? "VIDEO" : "AUDIO");
  return true;
}

bool MPVPlayer::configure_video_player()
{
  RCLCPP_DEBUG(logger_, "[%s] Configuring VIDEO player", config_.name.c_str());

  if (mpv_set_option_string(mpv_handle_, "keep-open", "yes") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'keep-open=yes'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "idle", "yes") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'idle=yes'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "audio-client-name", config_.name.c_str()) < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'audio-client-name=%s'", config_.name.c_str(), config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "ao", "null") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'ao=null'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "aid", "no") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'aid=no'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "vo", "drm") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'vo=drm'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "hwdec", "auto") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'hwdec=drm'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "image-display-duration", "86400") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'image-display-duration=86400'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "force-window", "yes") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'force-window=yes'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "video-rotate", config_.rotate.c_str()) < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'video-rotate=%s'", config_.name.c_str(), config_.rotate.c_str());
    return false;
  }

  // mpv_set_option_string(mpv_handle_, "loop-file", "no");

  return true;
}

bool MPVPlayer::configure_audio_player()
{
  RCLCPP_DEBUG(logger_, "[%s] Configuring AUDIO player", config_.name.c_str());

  if (mpv_set_option_string(mpv_handle_, "keep-open", "yes") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'keep-open=yes'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "idle", "yes") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'idle=yes'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "audio-client-name", config_.name.c_str()) < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'audio-client-name=%s'", config_.name.c_str(), config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "ao", "alsa") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'ao=alsa'", config_.name.c_str());
    return false;
  }

  std::string device = find_audio_device("Array");
  RCLCPP_INFO(logger_, "[%s] Auto-detected audio device: %s", config_.name.c_str(), device.c_str());

  if (mpv_set_option_string(mpv_handle_, "audio-device", device.c_str()) < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'audio-device'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "aid", "auto") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'aid=auto'", config_.name.c_str());
    return false;
  }

  if (mpv_set_option_string(mpv_handle_, "vo", "null") < 0)
  {
    RCLCPP_FATAL(logger_, "[%s] Failed to set option 'vo=null'", config_.name.c_str());
    return false;
  }

  // Дополнительные настройки для аудио
  // mpv_set_option_string(mpv_handle_, "gapless-audio", "yes");
  // mpv_set_option_string(mpv_handle_, "audio-pitch-correction", "no");

  return true;
}

void MPVPlayer::event_loop()
{
  RCLCPP_DEBUG(logger_, "[%s] Event loop started", config_.name.c_str());

  while (running_.load() && mpv_handle_)
  {
    mpv_event *ev = mpv_wait_event(mpv_handle_, config_.event_wait_timeout_ms / 1000.0);

    if (!ev || ev->event_id == MPV_EVENT_NONE)
      continue;

    switch (ev->event_id)
    {
    case MPV_EVENT_SHUTDOWN:
      RCLCPP_WARN(logger_, "[%s] MPV shutdown event received", config_.name.c_str());
      running_.store(false);
      break;

    case MPV_EVENT_FILE_LOADED:
      RCLCPP_DEBUG(logger_, "[%s] File loaded — playback started", config_.name.c_str());
      break;

    case MPV_EVENT_END_FILE:
      RCLCPP_DEBUG(logger_, "[%s] End of file reached", config_.name.c_str());
      break;

    default:
      break;
    }
  }

  RCLCPP_DEBUG(logger_, "[%s] Event loop terminated", config_.name.c_str());
}

int MPVPlayer::safe_command(const std::vector<const char *> &args)
{
  if (!mpv_handle_)
  {
    RCLCPP_ERROR(logger_, "[%s] Cannot execute command: mpv_handle is null", config_.name.c_str());
    return -1;
  }

  std::vector<const char *> argv = args;
  argv.push_back(nullptr);

  int r = mpv_command(mpv_handle_, (argv.data()));
  if (r < 0)
  {
    RCLCPP_DEBUG(logger_, "[%s] mpv_command '%s' returned error %d",
                 config_.name.c_str(), argv[0] ? argv[0] : "null", r);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(config_.command_timeout_ms));
  return r;
}

bool MPVPlayer::check_file_exists(const std::string &path) const
{
  if (path.empty())
    return false;

  // === Обработка интернет-потоков (пропускаем проверку) ===
  // Поддерживаемые протоколы: http(s), rtsp, rtmp, mms, hls, dash
  static const std::vector<std::string> stream_protocols = {
    "http://", "https://", "rtsp://", "rtmp://", 
    "mms://", "mmsh://", "mmst://", "mmsu://",
    "hls://", "dash://", "ytdl://"
  };
  
  for (const auto& proto : stream_protocols) {
    if (path.compare(0, proto.length(), proto) == 0) {
      RCLCPP_DEBUG(logger_, "[%s] Detected stream URL (protocol: %s), skipping file check", 
                   config_.name.c_str(), proto.c_str());
      return true; // Доверяем URL, проверка будет в mpv
    }
  }

  // === Обработка специальных путей ===
  // /dev/shm — временная память (мы сами пишем туда кадры)
  if (path.find("/dev/shm/") == 0) {
    return true;
  }
  
  // pipe:// — именованные каналы (используются для потоковой передачи)
  if (path.find("pipe://") == 0) {
    return true;
  }

  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0);
}

bool MPVPlayer::play(const std::string &path, bool loop)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!mpv_handle_)
  {
    RCLCPP_ERROR(logger_, "[%s] Cannot play: player not initialized", config_.name.c_str());
    return false;
  }

  mpv_set_property_string(mpv_handle_, "audio-format", "auto");
  mpv_set_property_string(mpv_handle_, "audio-samplerate", "auto");
  mpv_set_property_string(mpv_handle_, "audio-channels", "auto");

  mpv_set_property_string(mpv_handle_, "demuxer", "");

  if (!check_file_exists(path))
  {
    RCLCPP_ERROR(logger_, "[%s] File not found: %s", config_.name.c_str(), path.c_str());
    return false;
  }

  // Останавливаем текущее воспроизведение
  if (safe_command({"stop"}) < 0)
  {
    RCLCPP_ERROR(logger_, "[%s] Failed to stop playback", config_.name.c_str());
  }

  if (mpv_set_property_string(mpv_handle_, "loop-file", loop ? "inf" : "no") < 0)
  {
    RCLCPP_ERROR(logger_, "[%s] Failed to loop playback (%s)", config_.name.c_str(), path.c_str());
  }

  // Загружаем новый файл
  if (safe_command({"loadfile", path.c_str(), "replace"}) < 0)
  {
    RCLCPP_ERROR(logger_, "[%s] Failed to load file: %s", config_.name.c_str(), path.c_str());
    return false;
  }

  if (mpv_set_property_string(mpv_handle_, "pause", "no") < 0)
  {
    RCLCPP_ERROR(logger_, "[%s] Failed to unpause playback", config_.name.c_str());
  }

  // RCLCPP_INFO(logger_, "[%s] Started playback: %s", config_.name.c_str(), path.c_str());
  return true;
}


bool MPVPlayer::update_frame(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!mpv_handle_) {
        RCLCPP_ERROR(logger_, "[%s] Cannot update frame: player not initialized", config_.name.c_str());
        return false;
    }
    
    if (!check_file_exists(path)) {
        RCLCPP_ERROR(logger_, "[%s] Frame file not found: %s", config_.name.c_str(), path.c_str());
        return false;
    }
    
    // КРИТИЧЕСКИ ВАЖНО: Заменяем файл БЕЗ остановки воспроизведения
    if (safe_command({"loadfile", path.c_str(), "replace"}) < 0) {
        RCLCPP_ERROR(logger_, "[%s] Failed to update frame: %s", config_.name.c_str(), path.c_str());
        return false;
    }
    
    // Гарантируем, что воспроизведение не на паузе
    if (mpv_set_property_string(mpv_handle_, "pause", "no") < 0) {
        RCLCPP_WARN(logger_, "[%s] Failed to unpause after frame update", config_.name.c_str());
    }
    
    return true;
}



double MPVPlayer::set_volume(double volume)
{
  double volume_ = std::clamp(volume, 0.0, 100.0);

  int success = mpv_set_property(mpv_handle_, "volume", MPV_FORMAT_DOUBLE, &volume_);
  if (success >= 0)
  {
    RCLCPP_DEBUG(logger_, "[%s] Volume set to %f", config_.name.c_str(), volume_);
    return volume_;
  }
  else
  {
    RCLCPP_WARN(logger_, "[%s] Failed to set volume to %f", config_.name.c_str(), volume_);
    return -1;
  }
}

double MPVPlayer::get_volume() const
{
  if (!mpv_handle_)
    return -1;

  double vol = 0.0;
  if (mpv_get_property(mpv_handle_, "volume", MPV_FORMAT_DOUBLE, &vol) >= 0)
  {
    return std::clamp(vol, 0.0, 100.0);
  }
  return -1;
}

bool MPVPlayer::is_eof()
{
  if (!mpv_handle_)
    return false;

  if (running_.load())
  {

    int64_t eof = 0;
    if (mpv_get_property(mpv_handle_, "eof-reached", MPV_FORMAT_FLAG, &eof) >= 0 && eof)
    {
      return true;
    }
  }

  return false;
}

bool MPVPlayer::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!mpv_handle_) return false;
  
  bool result = (safe_command({"stop"}) >= 0);
  
  if (result) {
    RCLCPP_DEBUG(logger_, "[%s] Playback stopped", config_.name.c_str());
  } else {
    RCLCPP_WARN(logger_, "[%s] Failed to stop playback", config_.name.c_str());
  }
  
  return result;
}

bool MPVPlayer::is_idle() const {
    if (!mpv_handle_) return false;

    // 1. Проверяем: не в режиме ли ожидания (idle)
    int64_t idle = 0;
    if (mpv_get_property(mpv_handle_, "idle-active", MPV_FORMAT_FLAG, &idle) >= 0 && idle) {
        // RCLCPP_DEBUG(logger_, "[%s] idle-active=yes → ничего не воспроизводится", name_.c_str());
        return true;
    }

    // 2. Проверяем: не на паузе ли
    int64_t paused = 0;
    if (mpv_get_property(mpv_handle_, "pause", MPV_FORMAT_FLAG, &paused) >= 0 && paused) {
        // RCLCPP_DEBUG(logger_, "[%s] pause=yes → воспроизведение на паузе", name_.c_str());
        return true;
    }

    // 3. Проверяем: не достигнут ли конец файла
    int64_t eof = 0;
    if (mpv_get_property(mpv_handle_, "eof-reached", MPV_FORMAT_FLAG, &eof) >= 0 && eof) {
        // RCLCPP_DEBUG(logger_, "[%s] eof-reached=yes → воспроизведение завершено", name_.c_str());
        return true;
    }

    // 4. Дополнительно: проверяем, что файл вообще загружен
    char* path = nullptr;
    if (mpv_get_property(mpv_handle_, "path", MPV_FORMAT_STRING, &path) >= 0) {
        bool has_path = (path && path[0] != '\0');
        mpv_free(path);
        if (!has_path) {
            // RCLCPP_DEBUG(logger_, "[%s] path is empty → ничего не загружено", name_.c_str());
            return true;
        }
    }

    // Все проверки пройдены → активное воспроизведение
    return false;
}


bool MPVPlayer::play_from_pipe(const std::string& pipe_path, int sample_rate, int channels)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!mpv_handle_) {
        RCLCPP_ERROR(logger_, "[%s] Cannot play from pipe: not initialized", config_.name.c_str());
        return false;
    }

    safe_command({"stop"});

    mpv_set_property_string(mpv_handle_, "demuxer-rawaudio-rate", std::to_string(sample_rate).c_str());
    mpv_set_property_string(mpv_handle_, "demuxer-rawaudio-channels", std::to_string(channels).c_str());

    mpv_set_property_string(mpv_handle_, "demuxer", "rawaudio");
    mpv_set_property_string(mpv_handle_, "demuxer-rawaudio-format", "s16le"); // S16_LE

    mpv_set_property_string(mpv_handle_, "demuxer-readahead-secs", "0");
    mpv_set_property_string(mpv_handle_, "cache-secs", "0");
    mpv_set_property_string(mpv_handle_, "force-window", "no");

    // Очень важно: запретить mpv закрывать демуксер при временном отсутствии данных
    mpv_set_property_string(mpv_handle_, "keep-open", "always");
    
 
    std::string url = "file://" + pipe_path;


    int r = safe_command({"loadfile", url.c_str(), "replace"});

    if (r < 0) {
        RCLCPP_ERROR(logger_, "[%s] MPV loadfile failed: code=%d, path=%s", 
                     config_.name.c_str(), r, url.c_str());
        return false;
    }

    mpv_set_property_string(mpv_handle_, "pause", "no");
    RCLCPP_INFO(logger_, "[%s] Audio stream started: %s @ %dHz, %dch", 
                config_.name.c_str(), url.c_str(), sample_rate, channels);
    return true;
}