/*
ros2 service call /ears_driver/ears_set_angle robohead_interfaces/srv/Move "angle_a: 0
angle_b: 0
duration: 1.0
*/

/*
colcon build --symlink-install --packages-select robohead_interfaces ears_driver
*/

#include <rclcpp/rclcpp.hpp>
#include <robohead_interfaces/srv/move.hpp>
#include <robohead_interfaces/srv/simple_command.hpp>
#include <thread>
#include <cmath>
#include <chrono>
#include <mutex>
#include <iostream>

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06

class EarsDriver : public rclcpp::Node
{
    int _i2c_address;
    int _servo_1_channel;
    int _servo_2_channel;
    int _servo_1_coef;
    int _servo_2_coef;
    int _servo_pulse_low, _servo_pulse_high;
    int _l_from, _l_to, _r_from, _r_to;

    // Состояние
    std::mutex _goal_mutex;

    std::array<double, 2> _current_angles;
    std::array<double, 3> _goal_angles;

    rclcpp::Service<robohead_interfaces::srv::Move>::SharedPtr _srv_ears_set_angle;
    rclcpp::Service<robohead_interfaces::srv::SimpleCommand>::SharedPtr _srv_is_idle;

    std::thread _trajectory_thread;

    struct CubicCoeffs
    {
        double a0, a1, a2, a3;
    };

public:
    EarsDriver() : Node("ears_driver")
    {
        // Параметры
        std::string srv_ears_set_angle_name = this->declare_parameter<std::string>("srv_ears_set_angle_name", "ears_set_angle");
        std::string srv_is_idle_name = this->declare_parameter<std::string>("srv_is_idle_name", "is_idle");

        int std_l = this->declare_parameter<int>("std_left_angle", 0);
        int std_r = this->declare_parameter<int>("std_right_angle", 0);
        _i2c_address = this->declare_parameter<int>("i2c_address", 0x40);
        _servo_1_channel = this->declare_parameter<int>("servo_1_channel", 7);
        _servo_2_channel = this->declare_parameter<int>("servo_2_channel", 6);
        _servo_1_coef = this->declare_parameter<int>("servo_1_coef", 0);
        _servo_2_coef = this->declare_parameter<int>("servo_2_coef", 0);
        _servo_pulse_low = this->declare_parameter<int>("servo_pulse_low", 120);
        _servo_pulse_high = this->declare_parameter<int>("servo_pulse_high", 550);
        _l_from = this->declare_parameter<int>("constraints.l_from", -30);
        _l_to = this->declare_parameter<int>("constraints.l_to", 30);
        _r_from = this->declare_parameter<int>("constraints.r_from", -30);
        _r_to = this->declare_parameter<int>("constraints.r_to", 30);

        _goal_angles = {static_cast<double>(std_l), static_cast<double>(std_r), 0.0};
        pca9685_init(_i2c_address);

        {
            double angle1 = 90 + std_l + _servo_1_coef;
            double angle2 = 90 - std_r - _servo_2_coef;
            angle1 = std::clamp(angle1, 0.0, 180.0);
            angle2 = std::clamp(angle2, 0.0, 180.0);

            set_servo_angle(_servo_1_channel, angle1);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            set_servo_angle(_servo_2_channel, angle2);
            _current_angles = {static_cast<double>(std_l), static_cast<double>(std_r)};
        }

        _srv_ears_set_angle = this->create_service<robohead_interfaces::srv::Move>(
            srv_ears_set_angle_name,
            std::bind(&EarsDriver::handle_service_ears_set_angle, this, std::placeholders::_1, std::placeholders::_2));

        _srv_is_idle = this->create_service<robohead_interfaces::srv::SimpleCommand>(
            srv_is_idle_name,
            std::bind(&EarsDriver::handle_service_is_idle, this, std::placeholders::_1, std::placeholders::_2));

        _trajectory_thread = std::thread(&EarsDriver::trajectory_planner, this);

        RCLCPP_INFO(this->get_logger(), "INITED");
    }

    ~EarsDriver()
    {
        if (_trajectory_thread.joinable())
        {
            _trajectory_thread.join();
        }
        if (i2c_fd >= 0)
            close(i2c_fd);
    }

private:
    void set_angle(double left, double right)
    {
        std::lock_guard<std::mutex> lock(_goal_mutex);

        double angle1 = 90 + left + _servo_1_coef;
        double angle2 = 90 - right - _servo_2_coef;

        angle1 = std::clamp(angle1, 0.0, 180.0);
        angle2 = std::clamp(angle2, 0.0, 180.0);

        set_servo_angle(_servo_1_channel, angle1);
        set_servo_angle(_servo_2_channel, angle2);
        _current_angles = {left, right};
    }

    CubicCoeffs generate_cubic(double angle_cur, double angle_goal, double time)
    {
        if (time <= 0.0)
            time = 0.01;
        double a0 = angle_cur;
        double a1 = 0.0;
        double a2 = (3.0 / (time * time)) * (angle_goal - angle_cur);
        double a3 = (-2.0 / (time * time * time)) * (angle_goal - angle_cur);
        return {a0, a1, a2, a3};
    }

    void trajectory_planner()
    {
        double prev_goal_l = _goal_angles[0];
        double prev_goal_r = _goal_angles[1];

        std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
        std::chrono::time_point<std::chrono::steady_clock> now;
        rclcpp::Rate loop_rate(50); // 50 Hz
        double cur_l;
        double cur_r;
        double goal_l;
        double goal_r;
        double duration;
        CubicCoeffs cubic_l, cubic_r;
        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(_goal_mutex);
                cur_l = _current_angles[0];
                cur_r = _current_angles[1];
                goal_l = _goal_angles[0];
                goal_r = _goal_angles[1];
                duration = _goal_angles[2];
            }

            if (duration > 0.0)
            {
                if (prev_goal_l != goal_l || prev_goal_r != goal_r)
                {
                    start_time = std::chrono::steady_clock::now();
                    cubic_l = generate_cubic(cur_l, goal_l, duration);
                    cubic_r = generate_cubic(cur_r, goal_r, duration);
                    prev_goal_l = goal_l;
                    prev_goal_r = goal_r;
                }

                now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - start_time).count();

                double dist = std::sqrt(std::pow(goal_l - cur_l, 2) + std::pow(goal_r - cur_r, 2));

                if (dist >= 0.5 && elapsed < duration)
                {
                    set_angle(cubic_l.a0 + cubic_l.a2 * elapsed * elapsed + cubic_l.a3 * elapsed * elapsed * elapsed,
                              cubic_r.a0 + cubic_r.a2 * elapsed * elapsed + cubic_r.a3 * elapsed * elapsed * elapsed);
                }
                else
                {
                    set_angle(goal_l, goal_r);
                }
            }
            else
            {
                if (prev_goal_l != goal_l || prev_goal_r != goal_r)
                {
                    set_angle(goal_l, goal_r);
                    prev_goal_l = goal_l;
                    prev_goal_r = goal_r;
                }
            }

            loop_rate.sleep();
        }
    }

    void handle_service_is_idle(
        const std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Request> request,
        std::shared_ptr<robohead_interfaces::srv::SimpleCommand::Response> response)
    {
        std::lock_guard<std::mutex> lock(_goal_mutex);
        response->data = (_current_angles[0] == _goal_angles[0] && _current_angles[1] == _goal_angles[1]);
        // RCLCPP_ERROR(this->get_logger(), "Idle cmp %d: Current %f %f, goal %f %f", res, _current_angles[0],
                        // _current_angles[1], _goal_angles[0], _goal_angles[1]);
    }

    void handle_service_ears_set_angle(
        const std::shared_ptr<robohead_interfaces::srv::Move::Request> request,
        std::shared_ptr<robohead_interfaces::srv::Move::Response> response)
    {
        int16_t left = request->angle_a;
        int16_t right = request->angle_b;
        double duration = request->duration;

        if (left < _l_from || left > _l_to)
        {
            response->data = -1;
            return;
        }
        if (right < _r_from || right > _r_to)
        {
            response->data = -2;
            return;
        }
        if (duration < 0.0)
        {
            response->data = -3;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(_goal_mutex);
            _goal_angles = {static_cast<double>(left), static_cast<double>(right), duration};
        }

        response->data = 0;
    }

    bool i2c_write_byte(int fd, uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        return write(fd, buf, 2) == 2;
    }

    void pca9685_set_pwm_freq(int fd, double freq_hz)
    {
        // Частота внутреннего тактирования PCA9685 — 25 МГц
        double prescale_val = 25000000.0 / (4096.0 * freq_hz) - 1.0;
        uint8_t prescale = static_cast<uint8_t>(std::round(prescale_val));

        // Чтение текущего MODE1 (опционально)
        // Но проще просто записать 0x10 (sleep)
        i2c_write_byte(fd, 0x00, 0x10); // Sleep mode
        usleep(5000);

        // Устанавливаем prescaler
        i2c_write_byte(fd, 0xFE, prescale);
        usleep(5000);

        // Возвращаемся в normal mode + restart
        i2c_write_byte(fd, 0x00, 0xA0); // 0x80 (restart) + 0x20 (auto-increment)
        usleep(5000);
    }

    void pca9685_init(int address = 0x40)
    {
        const char *i2c_device = "/dev/i2c-1";
        i2c_fd = open(i2c_device, O_RDWR);
        if (i2c_fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open I2C device");
            throw std::runtime_error("Cannot open I2C device");
        }
        if (ioctl(i2c_fd, I2C_SLAVE, address) < 0)
        {
            close(i2c_fd);
            RCLCPP_ERROR(this->get_logger(), "Cannot set I2C slave address");
            throw std::runtime_error("Cannot set I2C slave address");
        }

        // Настройка частоты 50 Гц
        pca9685_set_pwm_freq(i2c_fd, 50.0);
    }

    void set_pwm(int channel, int on, int off)
    {
        uint8_t reg = 0x06 + 4 * channel;
        // Записываем 4 байта: ON_L, ON_H, OFF_L, OFF_H
        uint8_t buf[5];
        buf[0] = reg;
        buf[1] = on & 0xFF;
        buf[2] = (on >> 8) & 0xFF;
        buf[3] = off & 0xFF;
        buf[4] = (off >> 8) & 0xFF;

        if (write(i2c_fd, buf, 5) != 5)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write PWM to channel %d", channel);
        }
    }

    // Конвертация угла (0–180) в PWM (обычно 150–600 для 50 Гц)
    void set_servo_angle(int channel, double angle_deg)
    {
        angle_deg = std::clamp(angle_deg, 0.0, 180.0);

        int pulse = static_cast<int>(_servo_pulse_low + (angle_deg / 180.0) * (_servo_pulse_high - _servo_pulse_low)); // Диапазон от 100 до 550 (включительно)
                                                                                                                       // pulse 79 - 594 (MG995)
                                                                                                                       // pulse 94 - 585 (sg90)
                                                                                                                       // pulse 94 - 581 (sg90)
                                                                                                                       // pulse 93 - 579 (sg90)
                                                                                                                       // Возьмем 120 - 550
        set_pwm(channel, 0, pulse);
    }
    int i2c_fd = -1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<EarsDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}