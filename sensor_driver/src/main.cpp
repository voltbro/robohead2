#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <string>
#include <limits>

/*
colcon build --packages-select sensor_driver --symlink-install
*/

// === INA219 Constants ===
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_SHUNTVOLTAGE = 0x01;
constexpr uint8_t REG_BUSVOLTAGE = 0x02;
constexpr uint8_t REG_POWER = 0x03;
constexpr uint8_t REG_CURRENT = 0x04;
constexpr uint8_t REG_CALIBRATION = 0x05;

namespace BusVoltageRange {
    constexpr uint16_t RANGE_16V = 0x00;
    constexpr uint16_t RANGE_32V = 0x01;
}

namespace Gain {
    constexpr uint16_t DIV_1_40MV = 0x00;
    constexpr uint16_t DIV_2_80MV = 0x01;
    constexpr uint16_t DIV_4_160MV = 0x02;
    constexpr uint16_t DIV_8_320MV = 0x03;
}

namespace ADCResolution {
    constexpr uint16_t ADCRES_12BIT_32S = 0x0D;
}

namespace Mode {
    constexpr uint16_t SANDBVOLT_CONTINUOUS = 0x07;
}

// === INA219 Class (встроенный) ===
class INA219
{
public:
    explicit INA219(int i2c_bus = 1, uint8_t addr = 0x43)
        : addr_(addr), current_lsb_(0.1524f), cal_value_(26868)
    {
        std::string device = "/dev/i2c-" + std::to_string(i2c_bus);
        i2c_fd_ = open(device.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            throw std::runtime_error("Failed to open I2C device");
        }

        if (ioctl(i2c_fd_, I2C_SLAVE, addr_) < 0) {
            close(i2c_fd_);
            throw std::runtime_error("Failed to set I2C slave address");
        }

        setCalibration_16V_5A();
    }

    ~INA219()
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
    }

    float getBusVoltage_V()
    {
        writeRegister(REG_CALIBRATION, cal_value_);
        uint16_t value = readRegister(REG_BUSVOLTAGE);
        return static_cast<float>((value >> 3) * 0.004);
    }

    float getCurrent_mA()
    {
        uint16_t value = readRegister(REG_CURRENT);
        int16_t signed_value = static_cast<int16_t>(value);
        return static_cast<float>(signed_value * current_lsb_);
    }

private:
    void writeRegister(uint8_t reg, uint16_t value)
    {
        uint8_t buf[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
        write(i2c_fd_, buf, 3);
    }

    uint16_t readRegister(uint8_t reg)
    {
        write(i2c_fd_, &reg, 1);
        uint8_t buf[2];
        read(i2c_fd_, buf, 2);
        return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    }

    void setCalibration_16V_5A()
    {
        writeRegister(REG_CALIBRATION, cal_value_);

        uint16_t config = (BusVoltageRange::RANGE_16V << 13) |
                          (Gain::DIV_2_80MV << 11) |
                          (ADCResolution::ADCRES_12BIT_32S << 7) |
                          (ADCResolution::ADCRES_12BIT_32S << 3) |
                          Mode::SANDBVOLT_CONTINUOUS;
        
        writeRegister(REG_CONFIG, config);
    }

    int i2c_fd_ = -1;
    uint8_t addr_;
    float current_lsb_;
    uint16_t cal_value_;
};

// === ROS 2 Node ===
class SensorDriver : public rclcpp::Node
{
public:
    SensorDriver() : Node("sensor_driver")
    {
        this->declare_parameter<std::string>("topic_name", "~/battery");
        this->declare_parameter<int>("publish_rate", 5);
        this->declare_parameter<int>("i2c_address", 0x43);
        this->declare_parameter<int>("i2c_bus", 1);

        std::string topic_name = this->get_parameter("topic_name").as_string();
        int publish_rate = this->get_parameter("publish_rate").as_int();
        int i2c_address = this->get_parameter("i2c_address").as_int();
        int i2c_bus = this->get_parameter("i2c_bus").as_int();

        try {
            ina219_ = std::make_unique<INA219>(i2c_bus, static_cast<uint8_t>(i2c_address));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize INA219: %s", e.what());
            throw;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>(topic_name, 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate),
            std::bind(&SensorDriver::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "sensor_driver INITED");
    }

private:
    void timer_callback()
    {
        auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();
        msg->header.stamp = this->now();

        try {
            msg->voltage = ina219_->getBusVoltage_V();
            msg->current = ina219_->getCurrent_mA() / 1000.0f; // A
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "INA219 read error: %s", e.what());
            return;
        }

        // Заполняем остальные поля BatteryState
        msg->cell_voltage.clear();
        msg->location = "main_battery";
        msg->serial_number = "";
        msg->charge = std::numeric_limits<float>::quiet_NaN();
        msg->capacity = std::numeric_limits<float>::quiet_NaN();
        msg->design_capacity = std::numeric_limits<float>::quiet_NaN();
        msg->percentage = std::numeric_limits<float>::quiet_NaN();
        msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        msg->present = true;

        publisher_->publish(std::move(msg));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    std::unique_ptr<INA219> ina219_;
};

// === main ===
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}