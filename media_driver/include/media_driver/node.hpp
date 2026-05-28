#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "robohead_interfaces/srv/play_media.hpp"
#include "robohead_interfaces/srv/simple_command.hpp"
#include "robohead_interfaces/msg/audio_data.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "media_driver/mpv_player.hpp"
#include <algorithm>


class MediaDriverNode : public rclcpp::Node {
public:
  MediaDriverNode();
  ~MediaDriverNode();

private:
  void handle_stream_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void handle_stream_audio(const robohead_interfaces::msg::AudioData::SharedPtr msg);
  void handle_play_media(
    const std::shared_ptr<robohead_interfaces::srv::PlayMedia::Request> request,
    std::shared_ptr<robohead_interfaces::srv::PlayMedia::Response> response);
  void handle_set_volume(
    const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
    std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response);
  void handle_get_volume(
    const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
    std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response);
  void handle_is_idle_audio(
    const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
    std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response);
  void handle_is_idle_display(
    const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
    std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response);

  std::unique_ptr<MPV::MPVPlayer> video_player_;
  std::unique_ptr<MPV::MPVPlayer> audio_player_;
  std::atomic<bool> running_;
  std::mutex mtx_;
  std::string stop_command;

  rclcpp::Service<robohead_interfaces::srv::PlayMedia>::SharedPtr srv_play_;
  rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_set_vol_;
  rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_get_vol_;
  rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_is_idle_audio_;
  rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr srv_is_idle_display_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stream_;

  rclcpp::Subscription<robohead_interfaces::msg::AudioData>::SharedPtr sub_audio_stream_;
  std::string audio_fifo_path_ = "/dev/shm/robohead_audio.fifo";
  bool audio_stream_active_ = false;
  std::mutex audio_fifo_mutex_;
  int fifo_fd_ = -1;
};