// actuator_node.hpp
#ifndef ACTUATOR__ACTUATOR_NODE_HPP_
#define ACTUATOR__ACTUATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "teacar_msgs/msg/motioncmd.hpp"
#include <unordered_map>
#include <array>
#include <string>
#include <algorithm>

namespace actuator
{
class Actuator : public rclcpp::Node
{

public:
    Actuator(): Node("actuator_node")
    {
        // Declare and get parameter
        this->declare_parameter<int>("control_frequency", 50);
        this->get_parameter("control_frequency", control_frequency_);

        if (control_frequency_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid control_frequency %d, defaulting to 50", control_frequency_);
            control_frequency_ = 50;
        }

        // Create publisher and subscriber
    combined_cmd_pub_ = this->create_publisher<teacar_msgs::msg::Motioncmd>("/combined_motion_cmd", 10);

    cmd_sub_ = this->create_subscription<teacar_msgs::msg::Motioncmd>(
        "/motion_cmd", 10,
        std::bind(&Actuator::motion_callback, this, std::placeholders::_1));

    // Create timer
    timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_frequency_),
    std::bind(&Actuator::timer_callback, this));

        // Parameter update callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Actuator::on_parameter_change, this, std::placeholders::_1));

    }

protected:
virtual void actuate(float throttle, float steer) {}

private:
    int control_frequency_;
    rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr combined_cmd_pub_;
    rclcpp::Subscription<teacar_msgs::msg::Motioncmd>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::unordered_map<std::string, std::array<float, 2>> motion_cmds_;
    void timer_callback()
    {
        float combined_throttle = 0.0f;
        float combined_steer = 0.0f;

        for (const auto & pair : motion_cmds_) {
            combined_throttle += pair.second[0];
            combined_steer += pair.second[1];
          }

    // Use manual clamping instead of std::clamp for compatibility
    combined_throttle = std::max(-1.0f, std::min(1.0f, combined_throttle));
    combined_steer = std::max(-1.0f, std::min(1.0f, combined_steer));
        
        teacar_msgs::msg::Motioncmd msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "actuator";
        msg.throttle = combined_throttle;
        msg.steer = combined_steer;

        if (combined_cmd_pub_) {
            combined_cmd_pub_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Published combined cmd: throttle=%.2f, steer=%.2f", combined_throttle, combined_steer);
          }

        actuate(combined_throttle, combined_steer);


    }

    void motion_callback(const teacar_msgs::msg::Motioncmd::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received motion cmd: throttle=%.2f, steer=%.2f from node: %s",
                    msg->throttle, msg->steer, msg->header.frame_id.c_str());

        motion_cmds_[msg->header.frame_id][0] = msg->throttle;
        motion_cmds_[msg->header.frame_id][1] = msg->steer;
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto & param : parameters) {
            if (param.get_name() == "control_frequency") {
              int new_freq = param.as_int();
              if (new_freq > 0) {
                control_frequency_ = new_freq;
                timer_->reset();
                timer_->cancel();
                timer_ = this->create_wall_timer(
                  std::chrono::duration<double>(1.0 / control_frequency_),
                  std::bind(&Actuator::timer_callback, this));
      
                RCLCPP_INFO(this->get_logger(), "Updated control_frequency to %d Hz", control_frequency_);
              } else {
                RCLCPP_WARN(this->get_logger(), "Attempted to set invalid control_frequency %d", new_freq);
              }
            }
          }
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          result.reason = "Updated parameters successfully";
          return result;
    }


};
}

#endif  // ACTUATOR__ACTUATOR_NODE_HPP_