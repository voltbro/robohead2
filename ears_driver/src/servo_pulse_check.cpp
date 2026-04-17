// servo_pulse_check_refactored.cpp
// Standalone utility for PCA9685: set raw PWM (counts 0..4095) for selected channel.

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
constexpr uint8_t PCA9685_MODE1    = 0x00;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t LED0_ON_L        = 0x06;

constexpr int kMinChannel = 0;
constexpr int kMaxChannel = 15;

constexpr int kMinCount = 0;
constexpr int kMaxCount = 4095;

constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE1_AI      = 0x20;
constexpr uint8_t MODE1_SLEEP   = 0x10;

inline void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

std::runtime_error sys_error(const std::string& prefix)
{
  return std::runtime_error(prefix + ": " + std::strerror(errno));
}
}  // namespace

class I2CDevice
{
public:
  I2CDevice(std::string device, int address)
  : device_(std::move(device))
  {
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw sys_error("Cannot open I2C device " + device_);
    }

    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw sys_error("Cannot set I2C slave address (0x" + int_to_hex(address) + ")");
    }
  }

  ~I2CDevice()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  I2CDevice(const I2CDevice&) = delete;
  I2CDevice& operator=(const I2CDevice&) = delete;

  int fd() const { return fd_; }

  void write_byte(uint8_t reg, uint8_t value)
  {
    uint8_t buf[2] = {reg, value};
    if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
      throw sys_error("I2C write_byte failed (reg=0x" + int_to_hex(reg) + ")");
    }
  }

  uint8_t read_byte(uint8_t reg)
  {
    if (::write(fd_, &reg, 1) != 1) {
      throw sys_error("I2C select register failed (reg=0x" + int_to_hex(reg) + ")");
    }

    uint8_t value = 0;
    if (::read(fd_, &value, 1) != 1) {
      throw sys_error("I2C read_byte failed (reg=0x" + int_to_hex(reg) + ")");
    }
    return value;
  }

  void write_block(uint8_t start_reg, const uint8_t* data, size_t size)
  {
    // buffer: [start_reg][data...]
    std::string buf;
    buf.resize(1 + size);
    buf[0] = static_cast<char>(start_reg);
    std::memcpy(buf.data() + 1, data, size);

    if (::write(fd_, buf.data(), buf.size()) != static_cast<ssize_t>(buf.size())) {
      throw sys_error("I2C write_block failed (start_reg=0x" + int_to_hex(start_reg) + ")");
    }
  }

private:
  static std::string int_to_hex(int v)
  {
    static const char* digits = "0123456789ABCDEF";
    std::string s;
    s.push_back(digits[(v >> 4) & 0xF]);
    s.push_back(digits[v & 0xF]);
    return s;
  }

  std::string device_;
  int fd_ = -1;
};

class PCA9685
{
public:
  PCA9685(std::string i2c_device = "/dev/i2c-1", int address = 0x40)
  : dev_(std::move(i2c_device), address)
  {}

  void set_pwm_freq(double freq_hz)
  {
    if (freq_hz <= 0.0) {
      throw std::invalid_argument("freq_hz must be > 0");
    }

    const double prescale_val = 25000000.0 / (4096.0 * freq_hz) - 1.0;
    const auto prescale = static_cast<uint8_t>(std::lround(prescale_val));

    const uint8_t old_mode = dev_.read_byte(PCA9685_MODE1);

    // sleep (restart bit must be 0 while sleeping)
    const uint8_t sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    dev_.write_byte(PCA9685_MODE1, sleep_mode);
    sleep_ms(5);

    // prescale
    dev_.write_byte(PCA9685_PRESCALE, prescale);
    sleep_ms(5);

    // wake + enable auto-increment
    const uint8_t wake_mode = (old_mode & ~MODE1_SLEEP) | MODE1_AI;
    dev_.write_byte(PCA9685_MODE1, wake_mode);
    sleep_ms(5);

    // restart (optional, but ok)
    dev_.write_byte(PCA9685_MODE1, wake_mode | MODE1_RESTART);
    sleep_ms(5);
  }

  void set_pwm(int channel, int on_count, int off_count)
  {
    validate_channel(channel);
    on_count  = std::clamp(on_count,  kMinCount, kMaxCount);
    off_count = std::clamp(off_count, kMinCount, kMaxCount);

    const uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * channel);

    // ON_L, ON_H, OFF_L, OFF_H
    uint8_t payload[4];
    payload[0] = static_cast<uint8_t>(on_count & 0xFF);
    payload[1] = static_cast<uint8_t>((on_count >> 8) & 0x0F);
    payload[2] = static_cast<uint8_t>(off_count & 0xFF);
    payload[3] = static_cast<uint8_t>((off_count >> 8) & 0x0F);

    dev_.write_block(reg, payload, sizeof(payload));
  }

  void set_servo_angle_deg(
      int channel,
      double angle_deg,
      int pulse_low_count = 120,
      int pulse_high_count = 550)
  {
    validate_channel(channel);

    if (pulse_low_count < kMinCount || pulse_low_count > kMaxCount ||
        pulse_high_count < kMinCount || pulse_high_count > kMaxCount ||
        pulse_low_count >= pulse_high_count)
    {
      throw std::invalid_argument("Invalid pulse_low/high range");
    }

    angle_deg = std::clamp(angle_deg, 0.0, 180.0);

    const double t = angle_deg / 180.0;
    const int pulse = static_cast<int>(std::lround(
      pulse_low_count + t * (pulse_high_count - pulse_low_count)));

    set_pwm(channel, 0, pulse);
  }

private:
  static void validate_channel(int channel)
  {
    if (channel < kMinChannel || channel > kMaxChannel) {
      throw std::out_of_range("channel must be in [0..15]");
    }
  }

  I2CDevice dev_;
};

int main()
{
  try
  {
    PCA9685 pca("/dev/i2c-1", 0x40);
    pca.set_pwm_freq(50.0);

    int channel = 0;
    std::cout << "Input servo channel [0..15]: ";
    if (!(std::cin >> channel)) {
      std::cerr << "Invalid channel input\n";
      return 1;
    }

    if (channel < 0 || channel > 15) {
      std::cerr << "Channel must be in [0..15]\n";
      return 1;
    }

    std::cout << "OK. Now input PWM 'off' count [0..4095]. Ctrl+D to exit.\n";

    while (true)
    {
      int pulse = 0;
      std::cout << "Input pulse: ";
      if (!(std::cin >> pulse)) {
        std::cout << "\nExit.\n";
        break;
      }

      pulse = std::clamp(pulse, 0, 4095);
      pca.set_pwm(channel, 0, pulse);
      std::cout << "Set channel=" << channel << " pulse=" << pulse << "\n";
    }

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}