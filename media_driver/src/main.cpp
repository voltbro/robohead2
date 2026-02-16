/*
colcon build --symlink-install --packages-select media_driver robohead_interfaces

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_video_file: ''
path_to_audio_file: 'http://chanson.hostingradio.ru:8041/chanson256.mp3'
loop: false"

ros2 service call /media_driver/play_media robohead_interfaces/srv/PlayMedia "path_to_video_file: '__STOP__'
path_to_audio_file: '__STOP__'
loop: false" 


*/

#include <rclcpp/rclcpp.hpp>
#include "media_driver/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MediaDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Fatal error in media_driver_node: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}