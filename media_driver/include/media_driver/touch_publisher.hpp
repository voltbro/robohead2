#ifndef TOUCH_PUBLISHER_HPP
#define TOUCH_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/input.h>
#include <map>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

#include "robohead_interfaces/msg/touch_event.hpp"

#define NBITS(x) ((((x)-1)/(sizeof(long)*8))+1)
#define test_bit(bit, array) ((array[bit/(sizeof(long)*8)] >> (bit%(sizeof(long)*8))) & 1)

namespace touch_publisher
{

struct SlotState
{
    int tracking_id = -1;
    int state = 2; // 0 - down, 1 - move, 2 - up

    int x = -1;
    int y = -1;

    int last_x = -1;
    int last_y = -1;
};

class TouchPublisher : public rclcpp::Node
{
public:
    TouchPublisher();
    ~TouchPublisher();

private:
    std::string findTouchDevice(const std::string& device_path, const std::string& device_name);
    std::string getDeviceName(const std::string& device_path);
    bool hasMultiTouch(const std::string& device_path);
    bool openDevice(const std::string& device_path);

    void readLoop();
    void processEvent(const struct input_event& ev);
    void publishTouch(int slot, const SlotState& state, const std::string& event_state);

    void rotatePoint(int in_x, int in_y, int &out_x, int &out_y) const;

    rclcpp::Publisher<robohead_interfaces::msg::TouchEvent>::SharedPtr touch_pub_;

    int touch_fd_;
    std::string device_path_;
    std::string device_name_;

    int current_slot_;
    std::map<int, SlotState> slot_states_;

    std::unique_ptr<std::thread> read_thread_;
    std::atomic<bool> running_;

    // rotation + ranges
    int rotation_deg_; // 0..359, clockwise
    int abs_x_min_, abs_x_max_;
    int abs_y_min_, abs_y_max_;
    int width_, height_;
};

} // namespace touch_publisher

#endif