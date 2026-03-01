#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "controller/controller.hpp"  // Your converted header
#include <string>
#include <stdexcept>
#include <vector>

class JoystickController : public controller::Controller{

    public:
    JoystickController()
    :controller::Controller("joystick_controller_node")
    {
        this->declare_parameter<int>("throttle_axis", 4);
        this->declare_parameter<int>("steer_axis", 0);

        this->get_parameter("throttle_axis", throttle_axis_);
        this->get_parameter("steer_axis", steer_axis_);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&JoystickController::on_parameter_change, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoystickController::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joystick axis %d is mapped to throttle, axis %d is mapped to steer",
        throttle_axis_, steer_axis_);
    }

    private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    int throttle_axis_;
    int steer_axis_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received joystick input!");
  
      float throttle = 0.0f;
      float steer = 0.0f;
      
        // Check if axis indices are within range
      if (throttle_axis_ >= static_cast<int>(msg->axes.size())) {
        RCLCPP_WARN(this->get_logger(),
          "Throttle axis index %d is out of bounds! Joy message has %zu axes.",
          throttle_axis_, msg->axes.size());
        return;
      }

      
      if (steer_axis_ >= static_cast<int>(msg->axes.size())) {
        RCLCPP_WARN(this->get_logger(),
          "Steer axis index %d is out of bounds! Joy message has %zu axes.",
          steer_axis_, msg->axes.size());
        return;
      }

      throttle = msg->axes.at(throttle_axis_);
      steer = msg->axes.at(steer_axis_);

      RCLCPP_INFO(this->get_logger(), "Sending motion: throttle = %.2f, steer = %.2f", throttle, steer);
        // Wrap control call in try-catch to catch any runtime exceptions
      try {
        this->control(throttle, steer);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in control(): %s", e.what());
      }
      
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto & param : parameters) {
          if (param.get_name() == "throttle_axis") {
            throttle_axis_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Updated throttle_axis to %d", throttle_axis_);
          } else if (param.get_name() == "steer_axis") {
            steer_axis_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Updated steer_axis to %d", steer_axis_);
          }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Joystick axis parameters updated";
        return result;
    }

};


// Main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}