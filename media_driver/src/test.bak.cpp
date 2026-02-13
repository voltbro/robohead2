#include "rclcpp/rclcpp.hpp"
#include "media_driver/mpv_player.hpp"
#include <thread>
#include <chrono>

class MPVTestNode : public rclcpp::Node
{
public:
  MPVTestNode() : Node("mpv_test_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting MPV player test...");

    MPV::Config video_cfg;
    video_cfg.type = MPV::Type::VIDEO;
    video_cfg.name = "mpv_video";
    video_cfg.rotate = "270";

    video_player_ = std::make_unique<MPV::MPVPlayer>(this->get_logger(), video_cfg);

    if (!video_player_->initialize())
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize video player!");
      rclcpp::shutdown();
      return;
    }

    MPV::Config audio_cfg;
    audio_cfg.type = MPV::Type::AUDIO;
    audio_cfg.name = "audio_mpv";
    // Создаём и инициализируем аудио-плеер
    audio_player_ = std::make_unique<MPV::MPVPlayer>(this->get_logger(), audio_cfg);

    if (!audio_player_->initialize())
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize audio player!");
      rclcpp::shutdown();
      return;
    }

    // Запускаем тест через таймер (даём время на инициализацию)
    test_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MPVTestNode::run_test, this));
  }

private:
  void run_test()
  {
    audio_player_->set_volume(0);

    if (!video_player_->play("/home/pi/1video_audio.mov"))
      RCLCPP_ERROR(this->get_logger(), "Video play FAILED");


    if (!audio_player_->play("/home/pi/1video_audio.mov"))
      RCLCPP_ERROR(this->get_logger(), "Audio play FAILED");
    
    int i = 0;
    while (!audio_player_->is_eof() || !video_player_->is_eof())
    {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    audio_player_->set_volume(i++);
    }
    

    // Завершаем ноду
    rclcpp::shutdown();
  }

  std::unique_ptr<MPV::MPVPlayer> video_player_;
  std::unique_ptr<MPV::MPVPlayer> audio_player_;
  rclcpp::TimerBase::SharedPtr test_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MPVTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
