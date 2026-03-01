#include "controller/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ParamControllerNode : public controller::Controller
{
public:
    ParamControllerNode() : controller::Controller("param_controller_node")
    {
        this->declare_parameter("throttle_value", 0.0);
        this->declare_parameter("steer_angle", 0.0);

        // Load parameters once at startup
        this->get_parameter("throttle_value", throttle_);
        this->get_parameter("steer_angle", steer_);

            // Apply the control once using initial values
        this->control(throttle_, steer_);

            // Set up parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParamControllerNode::on_parameter_change, this, std::placeholders::_1)
        );


        RCLCPP_INFO(this->get_logger(), "ParamControllerNode started with throttle=%.2f, steer=%.2f",
         throttle_, steer_);
    }


private:
    double throttle_;
    double steer_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto & param : parameters)
    {
      if (param.get_name() == "throttle_value") {
        throttle_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated throttle: %.2f", throttle_);
      }
      else if (param.get_name() == "steer_angle") {
        steer_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated steer: %.2f", steer_);
      }
    }

        // Send control signal every time a parameter changes
        this->control(throttle_, steer_);

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Updated parameters successfully";
        return result;
    }
};

// Entry point
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
