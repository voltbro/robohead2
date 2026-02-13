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