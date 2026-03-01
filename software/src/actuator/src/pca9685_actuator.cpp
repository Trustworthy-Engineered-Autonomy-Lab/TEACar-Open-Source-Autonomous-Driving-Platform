// pca9685_actuator_node.cpp
#include "actuator.hpp"  // Base class
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

using std::placeholders::_1;

class PCA9685
{
public:
    PCA9685() = default;
    ~PCA9685() { Close(); }

    bool Open(const std::string &device_name, int address = 0x40, int freq = 60)
    {
      fd_ = open(device_name.c_str(), O_RDWR);
      if (fd_ < 0) {
        error_ = "Failed to open I2C device: " + device_name;
        return false;
      }
      if (ioctl(fd_, I2C_SLAVE, address) < 0) {
        error_ = "Failed to get I2C access for: " + device_name;
        return false;
      }
      reset();

      return setPWMFreq(freq);
    }

    void Close()
    {
      if (fd_ >= 0) 
      {
        uint8_t value = 0;
        
        readReg(MODE1, &value);
        writeReg(MODE1, value | 0x10);

        close(fd_);
        fd_ = -1;
      }
    }

    bool setPWMFreq(int freq)
    {
      float prescaleval = 25000000.0 / (4096.0 * freq) - 1.0;
      uint8_t prescale = static_cast<uint8_t>(round(prescaleval));
  
      uint8_t oldmode = 0;
      if (!readReg(MODE1, &oldmode)) return false;
      uint8_t newmode = (oldmode & 0x7F) | 0x10;
      if (!writeReg(MODE1, newmode)) return false;

      if (!writeReg(PRESCALE, prescale)) return false;

      if (!writeReg(MODE1, oldmode)) return false;
      usleep(5000);
      return writeReg(MODE1, oldmode | 0x80);
    }

    bool setPWMChannel(int ch, float duty_cycle)
    {
      uint16_t off = static_cast<uint16_t>(4096 * duty_cycle);
      return writeReg(LED0_ON_L + 4 * ch, 0) &&
             writeReg(LED0_ON_H + 4 * ch, 0) &&
             writeReg(LED0_OFF_L + 4 * ch, off & 0xFF) &&
             writeReg(LED0_OFF_H + 4 * ch, off >> 8);
    }

    std::string getError() const { return error_; }
  
private:
    int fd_ = -1;
    std::string error_;
    static constexpr uint8_t MODE1 = 0x00, PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06, LED0_ON_H = 0x07;
    static constexpr uint8_t LED0_OFF_L = 0x08, LED0_OFF_H = 0x09;
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;

    bool reset()
    {
      uint8_t buf[2] = {0x00, 0x06};
      return write(fd_, buf, 2) == 2;
    }

    bool writeReg(uint8_t reg, uint8_t val)
    {
      uint8_t buf[2] = {reg, val};
      return write(fd_, buf, 2) == 2;
    }
  
    bool readReg(uint8_t reg, uint8_t *val)
    {
      return write(fd_, &reg, 1) == 1 && read(fd_, val, 1) == 1;
    }
};


class PCA9685ActuatorNode : public actuator::Actuator
{
public:
    PCA9685ActuatorNode() : actuator::Actuator()
    {
    this->declare_parameter("bus_device", std::string("/dev/i2c-1"));
    this->declare_parameter("pwm_frequency", 60);
    this->declare_parameter("steer_pwm_channel", 1);
    this->declare_parameter("throttle_pwm_channel", 0);
    this->declare_parameter("steer_min_pw", 1000);
    this->declare_parameter("steer_mid_pw", 1500);
    this->declare_parameter("steer_max_pw", 2000);
    this->declare_parameter("throttle_min_pw", 1000);
    this->declare_parameter("throttle_mid_pw", 1500);
    this->declare_parameter("throttle_max_pw", 2000);

    // Load once at startup
    this->get_parameter("bus_device", bus_device_);
    this->get_parameter("pwm_frequency", pwm_freq_);
    pca_ = std::make_unique<PCA9685>();

    if (!pca_->Open(bus_device_, 0x40, pwm_freq_)) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to init PCA9685: %s", pca_->getError().c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ Successfully initialized PCA9685");
    }
    }


protected:
    void actuate(float throttle, float steer) override
    {
      // RCLCPP_INFO(this->get_logger(), "Received: throttle = %.2f, steer = %.2f", throttle, steer);

        this->get_parameter("steer_pwm_channel", steer_ch_);
        this->get_parameter("throttle_pwm_channel", throttle_ch_);
        this->get_parameter("steer_min_pw", steer_min_);
        this->get_parameter("steer_mid_pw", steer_mid_);
        this->get_parameter("steer_max_pw", steer_max_);
        this->get_parameter("throttle_min_pw", throttle_min_);
        this->get_parameter("throttle_mid_pw", throttle_mid_);
        this->get_parameter("throttle_max_pw", throttle_max_);

        float steer_pw = steer >= 0 ? steer * (steer_max_ - steer_mid_) + steer_mid_
                                    : steer * (steer_mid_ - steer_min_) + steer_mid_;
        float throttle_pw = throttle >= 0 ? throttle * (throttle_max_ - throttle_mid_) + throttle_mid_
                                        : throttle * (throttle_mid_ - throttle_min_) + throttle_mid_;
        float cycle = 1000000.0f / pwm_freq_;

        float steer_duty = steer_pw / cycle;
        float throttle_duty = throttle_pw / cycle;

        RCLCPP_DEBUG(this->get_logger(), "Calculated PWM -> Throttle: %.2f%%, Steer: %.2f%%", throttle_duty * 100, steer_duty * 100);

        if (!pca_->setPWMChannel(throttle_ch_, throttle_duty)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write throttle PWM: %s", pca_->getError().c_str());
        }
        else
        {
          RCLCPP_DEBUG(this->get_logger(), "✅ Throttle PWM successfully written: duty = %.3f", throttle_duty);
        }

        if (!pca_->setPWMChannel(steer_ch_, steer_duty)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write steer PWM: %s", pca_->getError().c_str());
        }
        else
        {
          RCLCPP_DEBUG(this->get_logger(), "✅ Steer PWM successfully written: duty = %.3f", steer_duty);
        }
    }

private:
    std::unique_ptr<PCA9685> pca_;
    std::string bus_device_;
    int pwm_freq_ = 60;
    int steer_ch_, throttle_ch_;
    int steer_min_, steer_mid_, steer_max_;
    int throttle_min_, throttle_mid_, throttle_max_;
    

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCA9685ActuatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}