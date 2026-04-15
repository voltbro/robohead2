#include "media_driver/touch_publisher.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace touch_publisher
{

TouchPublisher::TouchPublisher()
    : Node("touch_publisher"),
      touch_fd_(-1),
      current_slot_(0),
      running_(false),
      rotation_deg_(0),
      abs_x_min_(0), abs_x_max_(4095),
      abs_y_min_(0), abs_y_max_(4095),
      width_(4096), height_(4096)
{
    // Параметры
    std::string device_name = this->declare_parameter<std::string>("device_name", "waveshare");
    std::string device_path = this->declare_parameter<std::string>("device_path", "/dev/input/");
    std::string topic_name  = this->declare_parameter<std::string>("topic_touchscreen_name", "touchscreen");
    int rot = stoi(this->declare_parameter<std::string>("display_rotate", "0"));

    RCLCPP_ERROR(this->get_logger(), "Rotate: %d", rot);
    int r = rot% 360;
    if (r < 0) r += 360;
    // rotation_deg_ = r;
    rotation_deg_ = (360 - r) % 360;

    // Publisher
    touch_pub_ = this->create_publisher<robohead_interfaces::msg::TouchEvent>(topic_name, 10);

    // Поиск и открытие устройства
    std::string found_device = findTouchDevice(device_path, device_name);

    if (found_device.empty() || !openDevice(found_device)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open touch device!");
        throw std::runtime_error("Cannot open touch device");
    }

    RCLCPP_INFO(this->get_logger(), "Touch device: %s", device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Path: %s", device_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "rotation_deg: %d", rotation_deg_);

    // Запуск потока чтения
    running_ = true;
    read_thread_ = std::make_unique<std::thread>(&TouchPublisher::readLoop, this);
}

TouchPublisher::~TouchPublisher()
{
    running_ = false;
    if (read_thread_ && read_thread_->joinable()) {
        read_thread_->join();
    }
    if (touch_fd_ >= 0) {
        close(touch_fd_);
    }
}

std::string TouchPublisher::findTouchDevice(const std::string& device_path,
                                            const std::string& device_name)
{
    DIR* dir = opendir(device_path.c_str());
    if (!dir) return "";

    struct dirent* entry;
    std::string found;

    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.find("event") != 0) continue;

        std::string full_path = device_path + filename;
        std::string dev_name = getDeviceName(full_path);

        // Case-insensitive поиск
        std::string dev_lower = dev_name;
        std::string search_lower = device_name;
        std::transform(dev_lower.begin(), dev_lower.end(), dev_lower.begin(), ::tolower);
        std::transform(search_lower.begin(), search_lower.end(), search_lower.begin(), ::tolower);

        if (dev_lower.find(search_lower) != std::string::npos && hasMultiTouch(full_path)) {
            RCLCPP_INFO(this->get_logger(), "Found: %s", dev_name.c_str());
            found = full_path;
            break;
        }
    }

    closedir(dir);
    return found;
}

std::string TouchPublisher::getDeviceName(const std::string& device_path)
{
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) return "";

    char name[256] = "Unknown";
    ioctl(fd, EVIOCGNAME(sizeof(name)), name);
    close(fd);
    return std::string(name);
}

bool TouchPublisher::hasMultiTouch(const std::string& device_path)
{
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) return false;

    unsigned long abs_bits[NBITS(ABS_MAX)];
    std::memset(abs_bits, 0, sizeof(abs_bits));

    if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        close(fd);
        return false;
    }

    bool has_mt = test_bit(ABS_MT_POSITION_X, abs_bits) &&
                  test_bit(ABS_MT_POSITION_Y, abs_bits);
    close(fd);
    return has_mt;
}

bool TouchPublisher::openDevice(const std::string& device_path)
{
    touch_fd_ = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (touch_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Open failed: %s", strerror(errno));
        return false;
    }

    device_path_ = device_path;
    device_name_ = getDeviceName(device_path);

    // Считываем диапазоны координат, чтобы корректно вращать
    input_absinfo abs{};

    // X
    if (ioctl(touch_fd_, EVIOCGABS(ABS_MT_POSITION_X), &abs) == 0) {
        abs_x_min_ = abs.minimum;
        abs_x_max_ = abs.maximum;
    } else if (ioctl(touch_fd_, EVIOCGABS(ABS_X), &abs) == 0) {
        abs_x_min_ = abs.minimum;
        abs_x_max_ = abs.maximum;
    } else {
        abs_x_min_ = 0; abs_x_max_ = 4095;
    }

    // Y
    if (ioctl(touch_fd_, EVIOCGABS(ABS_MT_POSITION_Y), &abs) == 0) {
        abs_y_min_ = abs.minimum;
        abs_y_max_ = abs.maximum;
    } else if (ioctl(touch_fd_, EVIOCGABS(ABS_Y), &abs) == 0) {
        abs_y_min_ = abs.minimum;
        abs_y_max_ = abs.maximum;
    } else {
        abs_y_min_ = 0; abs_y_max_ = 4095;
    }

    width_  = abs_x_max_ - abs_x_min_ + 1;
    height_ = abs_y_max_ - abs_y_min_ + 1;

    RCLCPP_INFO(this->get_logger(), "Touch range X:[%d..%d] Y:[%d..%d]",
                abs_x_min_, abs_x_max_, abs_y_min_, abs_y_max_);

    return true;
}

void TouchPublisher::readLoop()
{
    struct input_event ev;

    while (running_) {
        while (true) {
            ssize_t bytes = read(touch_fd_, &ev, sizeof(ev));

            if (bytes == sizeof(ev)) {
                processEvent(ev);
                continue;
            }

            if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break; // событий пока нет
            }

            if (bytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
                running_ = false;
                break;
            }

            // bytes == 0 или что-то странное
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void TouchPublisher::rotatePoint(int in_x, int in_y, int &out_x, int &out_y) const
{
    // делаем 0-based (на твоём устройстве min=0, но так надёжнее)
    int x = in_x - abs_x_min_;
    int y = in_y - abs_y_min_;

    int w = width_;
    int h = height_;

    int xr = x;
    int yr = y;

    if (rotation_deg_ == 0) {
        xr = x; yr = y;
    } else if (rotation_deg_ == 90) {        // 90 CW
        xr = (h - 1) - y;
        yr = x;
    } else if (rotation_deg_ == 180) {       // 180 CW
        xr = (w - 1) - x;
        yr = (h - 1) - y;
    } else if (rotation_deg_ == 270) {       // 270 CW
        xr = y;
        yr = (w - 1) - x;
    } else {
        double a  = rotation_deg_ * M_PI / 180.0;
        double cx = (w - 1) / 2.0;
        double cy = (h - 1) / 2.0;

        double dx = x - cx;
        double dy = y - cy;

        double ca = std::cos(a);
        double sa = std::sin(a);

        // clockwise
        double fx = dx * ca + dy * sa;
        double fy = -dx * sa + dy * ca;

        xr = static_cast<int>(std::lround(fx + cx));
        yr = static_cast<int>(std::lround(fy + cy));

        xr = std::clamp(xr, 0, w - 1);
        yr = std::clamp(yr, 0, h - 1);
    }

    out_x = xr;
    out_y = yr;
}

void TouchPublisher::processEvent(const struct input_event& ev)
{
    if (ev.type != EV_ABS && ev.type != EV_SYN) return;

    // выбор слота
    if (ev.type == EV_ABS && ev.code == ABS_MT_SLOT) {
        current_slot_ = ev.value;
        return;
    }

    // tracking id: down/up
    if (ev.type == EV_ABS && ev.code == ABS_MT_TRACKING_ID) {
        SlotState &st = slot_states_[current_slot_];

        if (ev.value >= 0) {
            st.tracking_id = ev.value;
            st.state = 0; // 0 - down
            st.last_x = -1;
            st.last_y = -1;
            st.x = -1;
            st.y = -1;

        } else {
            st.state = 2; // 2 - up
        }
        return;
    }

    // координаты
    if (ev.type == EV_ABS && ev.code == ABS_MT_POSITION_X) {
        SlotState &st = slot_states_[current_slot_];
        st.x = ev.value;
        if (st.last_x==-1) st.last_x = st.x;
        return;
    }

    if (ev.type == EV_ABS && ev.code == ABS_MT_POSITION_Y) {
        SlotState &st = slot_states_[current_slot_];
        st.y = ev.value;
        if (st.last_y==-1) st.last_y = st.y;
        return;
    }

    // конец фрейма
    if (ev.type == EV_SYN && ev.code == SYN_REPORT) {
        for (auto it = slot_states_.begin(); it != slot_states_.end(); ) {
            int slot = it->first;
            SlotState &st = it->second;

            // UP
            if (st.state==2) {
                publishTouch(slot, st, "up");
                it = slot_states_.erase(it);
                continue;
            }

            // DOWN 
            if (st.state==0) {
                if (st.x!=-1 && st.y!=-1) {
                    publishTouch(slot, st, "down");
                    st.state = 1; // move
                }
                ++it;
                continue;
            }

            // MOVE — если реально изменились координаты
            if (st.state==1 && (st.x != st.last_x || st.y != st.last_y)) {
                publishTouch(slot, st, "move");
                st.last_x = st.x;
                st.last_y = st.y;
            }
            ++it;
        }
    }
}

void TouchPublisher::publishTouch(int slot, const SlotState& state, const std::string& event_state)
{
    int rx = state.x;
    int ry = state.y;
    rotatePoint(state.x, state.y, rx, ry);

    robohead_interfaces::msg::TouchEvent msg;
    msg.slot = slot;
    msg.tracking_id = state.tracking_id; // на up остаётся прежним (не -1)
    msg.x = rx;
    msg.y = ry;
    msg.state = event_state;

    touch_pub_->publish(msg);
}

} // namespace touch_publisher

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<touch_publisher::TouchPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}