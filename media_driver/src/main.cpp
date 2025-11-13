// src/media_driver_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <filesystem>
#include <mpv/client.h>
//#include <mpv/opengl_cb.h> // not used, but keeps includes consistent
#include "robohead_interfaces/srv/play_media.hpp"
#include "robohead_interfaces/srv/simple_command.hpp"

using namespace std::chrono_literals;
using PlayMedia = robohead_interfaces::srv::PlayMedia;
using SimpleCommand = robohead_interfaces::srv::SimpleCommand;

static bool has_image_ext(const std::string &p) {
  std::string s = p;
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  if (s.size() < 4) return false;
  auto ends_with = [&](const char *ext) {
    size_t el = strlen(ext);
    return s.size() >= el && s.compare(s.size()-el, el, ext) == 0;
  };
  return ends_with(".png") || ends_with(".jpg") || ends_with(".jpeg") ||
         ends_with(".bmp") || ends_with(".webp");
}

class MediaDriverNode : public rclcpp::Node {
public:
  MediaDriverNode()
  : Node("media_driver"), mpv_handle_(nullptr), running_(true), playlist_index_(0)
  {
    // create mpv
    mpv_handle_ = mpv_create();
    if (!mpv_handle_) {
      RCLCPP_FATAL(this->get_logger(), "mpv_create() failed");
      throw std::runtime_error("mpv_create failed");
    }

    mpv_set_option_string(mpv_handle_, "vo", "drm"); // or gpu

    mpv_set_option_string(mpv_handle_, "ao", "alsa");        // или "pcm", "null"
    mpv_set_option_string(mpv_handle_, "hwdec", "auto");
    mpv_set_option_string(mpv_handle_, "keep-open", "yes");  // Не закрывать после конца
    mpv_set_option_string(mpv_handle_, "image-display-duration", "86400"); // 24 часа — "навсегда"
    mpv_set_option_string(mpv_handle_, "loop-file", "no");
    mpv_set_option_string(mpv_handle_, "idle", "yes");       // Разрешить пустой старт
    mpv_set_option_string(mpv_handle_, "force-window", "yes");
    mpv_set_option_string(mpv_handle_, "background", "black");
    mpv_set_option_string(mpv_handle_, "video-rotate", "270");


    // if (fullscreen_) {
    //   mpv_set_option_string(mpv_handle_, "fs", "yes");
    // }
    // initialize
    if (mpv_initialize(mpv_handle_) < 0) {
      RCLCPP_FATAL(this->get_logger(), "mpv_initialize() failed");
      mpv_terminate_destroy(mpv_handle_);
      mpv_handle_ = nullptr;
      throw std::runtime_error("mpv_initialize failed");
    }

    // services
    srv_play_ = this->create_service<PlayMedia>(
      "/robohead_controller/media_driver/play_media",
      std::bind(&MediaDriverNode::handle_play_media, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_set_vol_ = this->create_service<SimpleCommand>(
      "/robohead_controller/media_driver/set_volume",
      std::bind(&MediaDriverNode::handle_set_volume, this, std::placeholders::_1, std::placeholders::_2)
    );

    srv_get_vol_ = this->create_service<SimpleCommand>(
      "/robohead_controller/media_driver/get_volume",
      std::bind(&MediaDriverNode::handle_get_volume, this, std::placeholders::_1, std::placeholders::_2)
    );

    // event thread
    event_thread_ = std::thread(&MediaDriverNode::event_loop, this);

    RCLCPP_INFO(this->get_logger(), "MediaDriverNode initialized (libmpv)");
  }

  ~MediaDriverNode() override {
    running_.store(false);
    if (event_thread_.joinable()) event_thread_.join();
    if (mpv_handle_) {
      mpv_command_string(mpv_handle_, "stop");
      mpv_terminate_destroy(mpv_handle_);
      mpv_handle_ = nullptr;
    }
  }

private:
  // mpv handle
  mpv_handle *mpv_handle_;
  std::atomic<bool> running_;
  std::thread event_thread_;
  std::mutex mtx_;
  int image_fade_ms_;
  std::string hwdec_;
  bool fullscreen_;
  int wait_block_timeout_s_;
  int playlist_index_;
  std::atomic<bool> last_file_loaded_{false};


  rclcpp::Service<PlayMedia>::SharedPtr srv_play_;
  rclcpp::Service<SimpleCommand>::SharedPtr srv_set_vol_;
  rclcpp::Service<SimpleCommand>::SharedPtr srv_get_vol_;

  // helper: safely call mpv_command and log error if negative
int safe_command(const std::vector<const char*> &argv) {
  if (!mpv_handle_) return -1;
  // mpv_command doesn't actually modify the strings, so const_cast is safe here
  int r = mpv_command(mpv_handle_,
                      const_cast<const char**>(argv.data()));
  if (r < 0) {
    RCLCPP_DEBUG(this->get_logger(), "mpv_command returned %d", r);
  }
  return r;
}

void event_loop() {
  while (running_.load() && mpv_handle_) {
    mpv_event *ev = mpv_wait_event(mpv_handle_, 0.2);
    if (!ev) break;
    switch (ev->event_id) {
      case MPV_EVENT_NONE:
        break;
      case MPV_EVENT_SHUTDOWN:
        running_.store(false);
        break;
      case MPV_EVENT_FILE_LOADED:
        RCLCPP_INFO(this->get_logger(), "mpv: FILE_LOADED — first frame displayed");
        last_file_loaded_.store(true);
        break;
      case MPV_EVENT_END_FILE:
        RCLCPP_DEBUG(this->get_logger(), "mpv: end file event");
        break;
      default:
        break;
    }
  }
}


  // // apply short fade filter for images (lavfi fade). clears filter after delay
  // void apply_fade_and_clear(int fade_ms) {
  //   if (!mpv_handle_) return;
  //   double fade_s = std::max(0.0, fade_ms / 1000.0);
  //   std::string filter = std::string("lavfi=[fade=in:st=0:d=") + std::to_string(fade_s) + "]";
  //   // vf add <filter>
  //   const char *argv1[] = {"vf", "add", filter.c_str(), nullptr};
  //   safe_command({argv1[0], argv1[1], argv1[2], nullptr});
  //   // spawn thread to clear vf after fade+small margin
  //   std::thread([this, fade_s](){
  //     std::this_thread::sleep_for(std::chrono::milliseconds(int((fade_s + 0.05) * 1000)));
  //     const char *argv2[] = {"vf", "clear", nullptr};
  //     std::lock_guard<std::mutex> lk(mtx_);
  //     safe_command({argv2[0], argv2[1], nullptr});
  //   }).detach();
  // }

  // Try to set property loop-file to "inf" or "no"
  void set_loop(bool inf) {
    if (!mpv_handle_) return;
    const char *argv[] = {"set_property", "loop-file", inf ? "inf" : "no", nullptr};
    safe_command({argv[0], argv[1], argv[2], nullptr});
  }

  // blocking wait until EOF or node shutdown
  void wait_until_eof_or_stop() {
    if (!mpv_handle_) return;
    auto t0 = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(wait_block_timeout_s_);
    while (rclcpp::ok() && running_.load()) {
      // property eof-reached (flag)
      int64_t eof_flag = 0;
      if (mpv_get_property(mpv_handle_, "eof-reached", MPV_FORMAT_FLAG, &eof_flag) >= 0) {
        if (eof_flag) break;
      } else {
        // fallback: small sleep and continue
      }
      if (std::chrono::steady_clock::now() - t0 > timeout) {
        RCLCPP_WARN(this->get_logger(), "wait_until_eof: timeout reached");
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
  }



  
  // PlayMedia handler
  void handle_play_media(
    const std::shared_ptr<PlayMedia::Request> request,
    std::shared_ptr<PlayMedia::Response> response)
  {
    response->data = -1;

    std::string path = request->path_to_media_file;
    std::string override_audio = request->path_to_override_audio_file;
    bool is_block = request->is_block;
    bool is_cycle = request->is_cycle;

    const char* stop_cmd[] = {"stop", nullptr};
    mpv_command(mpv_handle_, stop_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // If empty path -> stop
    if (path.empty()) {
      // std::lock_guard<std::mutex> lk(mtx_);
      // const char *stop_cmd[] = {"stop", nullptr};
      // safe_command(stop_cmd);
      response->data = 0;
      return;
    }




        const char* cmd[] = {"loadfile", path.c_str(), "replace", nullptr};
    mpv_command(mpv_handle_, cmd);
    if (!override_audio.empty()) {
        // mpv_set_property_string(mpv_handle_, "aid", "no");
        const char* cmd_audio_clr[] = {"audio-clr", nullptr};
        mpv_command(mpv_handle_, cmd_audio_clr);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const char* cmd_audio[] = {"audio-add", override_audio.c_str(), nullptr};
        mpv_command(mpv_handle_, cmd_audio);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        mpv_set_property_string(mpv_handle_, "aid", "auto");
    }
    mpv_set_property_string(mpv_handle_, "pause", "no");

    RCLCPP_WARN(this->get_logger(), "[media_driver] displayed: %s, audio: %s", path.c_str(), override_audio.c_str());
    response->data = 0;
  }

  // volume set
  void handle_set_volume(
    const std::shared_ptr<SimpleCommand::Request> request,
    std::shared_ptr<SimpleCommand::Response> response)
  {
    int32_t val = request->data;
    int32_t vol = std::max(0, std::min(100, (int)val));
    double v = static_cast<double>(vol);
    if (mpv_set_property(mpv_handle_, "volume", MPV_FORMAT_DOUBLE, &v) < 0) {
      RCLCPP_WARN(this->get_logger(), "set volume failed");
      response->data = -1;
    } else {
      response->data = static_cast<int16_t>(vol);
    }
  }

  // volume get
  void handle_get_volume(
    const std::shared_ptr<SimpleCommand::Request> /*request*/,
    std::shared_ptr<SimpleCommand::Response> response)
  {
    double vol = 0;
    if (mpv_get_property(mpv_handle_, "volume", MPV_FORMAT_DOUBLE, &vol) < 0) {
      response->data = -1;
    } else {
      int v = static_cast<int>(std::round(vol));
      v = std::max(0, std::min(100, v));
      response->data = static_cast<int16_t>(v);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MediaDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    return 2;
  }
  return 0;
}
