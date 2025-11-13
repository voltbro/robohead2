
#include <thread>
#include <algorithm> 
#include <cmath>
#include <chrono>
#include <iostream>

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06


class NeckDriver
{
    // int _i2c_address;
    int i2c_fd = -1;

public:
    NeckDriver() 
    {

    }

    ~NeckDriver()
    {
    
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

        // Переводим в sleep mode
        uint8_t old_mode = 0;
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
            // RCLCPP_ERROR(rclcpp::get_logger("move_driver"), "Cannot open I2C device");
            throw std::runtime_error("Cannot open I2C device");
        }
        if (ioctl(i2c_fd, I2C_SLAVE, address) < 0)
        {
            close(i2c_fd);
            // RCLCPP_ERROR(rclcpp::get_logger("move_driver"), "Cannot set I2C slave address");
            throw std::runtime_error("Cannot set I2C slave address");
        }

        // Настройка частоты 50 Гц
        pca9685_set_pwm_freq(i2c_fd, 50.0);
    }

    void set_pwm(int channel, int on, int off)
    {
        std::cout << "pulse: "<<off<<std::endl;
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
            // RCLCPP_ERROR(rclcpp::get_logger("move_driver"), "Failed to write PWM to channel %d", channel);
        }
    }

    // Конвертация угла (0–180) в PWM (обычно 150–600 для 50 Гц)
    void set_servo_angle(int channel, double angle_deg)
    {
        int _servo_pulse_high = 550;
        int _servo_pulse_low = 120;
        // static bool initialized = false;
        // if (i2c_fd==-1)
        // {
        //     pca9685_init(0x40);
        //     // initialized = true;
        // }
        // int pulse  = angle_deg;
        angle_deg = std::clamp(angle_deg, 0.0, 180.0);

        int pulse = static_cast<int>(_servo_pulse_low + (angle_deg / 180.0) * (_servo_pulse_high - _servo_pulse_low)); // Диапазон от 100 до 550 (включительно)
                                                                                                                       // pulse 79 - 594 (MG995)
                                                                                                                       // pulse 94 - 585 (sg90)
                                                                                                                       // pulse 94 - 581 (sg90)
                                                                                                                       // pulse 93 - 579 (sg90)
                                                                                                                       // Возьмем 120 - 550 - оптимально!
                                                                                                                       // DS3239mh 77-597
        set_pwm(channel, 0, pulse);
    }
};

int main(int argc, char *argv[])
{
    NeckDriver obj;
    obj.pca9685_init(0x40);
    int channel;
        std::cin >> channel;
    while (true)
    {
        int pulse;
        std::cin >> pulse;
        obj.set_pwm(channel, 0, pulse);
        // obj.set_servo_angle(channel, pulse);
    }
    return 0;
}