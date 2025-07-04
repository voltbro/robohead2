#include "rclcpp/rclcpp.hpp"
#include "robohead_msgs/srv/simple_command.hpp"                                       
#include "robohead_msgs/srv/play_file.hpp"                                       
#include <memory>
#include <chrono>
#include <mpd/client.h>
#include <mpd/status.h>
#include <mpd/mixer.h>
#include <string.h>

#include <alsa/asoundlib.h>

/* example usage
ros2 service call /speakers_driver/play_audio robohead_msgs/srv/PlayFile "path_to_file: '/home/pi/file.mp3'
is_block: 0
is_cycle: 0" 
*/

void set_volume_callback(const std::shared_ptr<robohead_msgs::srv::SimpleCommand::Request> request,
                        std::shared_ptr<robohead_msgs::srv::SimpleCommand::Response>       response)
{
  if ((request->data < 0) || (request->data > 100))
  {
    response->data = -1;
  } else
  {
    struct mpd_connection *conn = mpd_connection_new(NULL, 0, 5000);
    int answer = mpd_run_set_volume(conn, request->data) ? 0 : -1;
    mpd_connection_free(conn);
    response->data = answer;
  }
}

void get_volume_callback(const std::shared_ptr<robohead_msgs::srv::SimpleCommand::Request> request,
                          std::shared_ptr<robohead_msgs::srv::SimpleCommand::Response>       response)
{
  struct mpd_connection *conn = mpd_connection_new(NULL, 0, 5000);
  int answer = mpd_run_get_volume(conn);
  mpd_connection_free(conn);
  response->data = answer;
}

void play_audio_callback(const std::shared_ptr<robohead_msgs::srv::PlayFile::Request> request,
                          std::shared_ptr<robohead_msgs::srv::PlayFile::Response>       response)
{
  struct mpd_connection *conn = mpd_connection_new(NULL, 0, 5000);
  mpd_run_stop(conn);
  mpd_run_clear(conn);

  int answer = mpd_run_add(conn, (request->path_to_file).c_str()) ? 0 : -1;
  mpd_run_repeat(conn, request->is_cycle);
  mpd_run_play(conn);
  
  while ((mpd_status_get_state(mpd_run_status(conn)) == MPD_STATE_PLAY) && request->is_block)
    rclcpp::sleep_for(std::chrono::nanoseconds(50'000'000)); // nanoseconds, 50'000'000 ns = 0.05 s = 20 hz

  mpd_connection_free(conn);
  response->data = answer;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("speakers_driver");

  node->declare_parameter("srv_set_volume_name", "~/set_volume");
  std::string srv_set_volume_name = (node->get_parameter("srv_set_volume_name")).as_string();

  node->declare_parameter("srv_get_volume_name", "~/get_volume");
  std::string srv_get_volume_name = (node->get_parameter("srv_get_volume_name")).as_string();

  node->declare_parameter("srv_play_audio_name", "~/play_audio");
  std::string srv_play_audio_name = (node->get_parameter("srv_play_audio_name")).as_string();
  
  node->declare_parameter("default_volume", 50);
  int default_volume = (node->get_parameter("default_volume")).as_int();

  mpd_run_set_volume(mpd_connection_new(NULL, 0, 5000), default_volume);

  rclcpp::Service<robohead_msgs::srv::SimpleCommand>::SharedPtr srv_set_volume = node->create_service<robohead_msgs::srv::SimpleCommand>(srv_set_volume_name,  &set_volume_callback);
  rclcpp::Service<robohead_msgs::srv::SimpleCommand>::SharedPtr srv_get_volume = node->create_service<robohead_msgs::srv::SimpleCommand>(srv_get_volume_name,  &get_volume_callback);
  rclcpp::Service<robohead_msgs::srv::PlayFile>::SharedPtr srv_play_audio = node->create_service<robohead_msgs::srv::PlayFile>(srv_play_audio_name,  &play_audio_callback);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "speakers_driver INITED");
  rclcpp::spin(node);
  rclcpp::shutdown();
}