#ifndef CONTROLLER__CONTROLLER_HPP_
#define CONTROLLER__CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "teacar_msgs/msg/motioncmd.hpp"
#include <cmath>
#include <string>

namespace controller
{
    class Controller: public rclcpp::Node
{
    public:
    Controller(const std::string & node_name): rclcpp::Node(node_name)
    {
        publisher_ = this->create_publisher<teacar_msgs::msg::Motioncmd>("/motion_cmd", 10);
        this->declare_parameter<double>("Throttle_ratio",1.0);
        this->declare_parameter<double>("steer_ratio",1.0);

        this->get_parameter("Throttle_ratio",throttle_ratio_);
        this->get_parameter("steer_ratio", steer_ratio_);
        
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        RCLCPP_INFO(this->get_logger(), " Controller %s Configuration", this->get_name());
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "%-20s | %-10s", "Parameter", "Value");
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "%-20s | %-10f", "Throttle ratio", throttle_ratio_);
        RCLCPP_INFO(this->get_logger(), "%-20s | %-10f", "Steer ratio", steer_ratio_);
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");

        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Controller::parameterCallback, 
                                                this, std::placeholders::_1));
    }
    
    void control(float throttle, float steer)
    {
        auto msg = teacar_msgs::msg::Motioncmd();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = this->get_name();

        if(throttle > 0)
        {
            msg.throttle = throttle_ratio_ * std::log( 1 + throttle * 1.71828);
        }
        else
        {
            msg.throttle = -throttle_ratio_ * std::log(1 - throttle * 1.71828);
        }

        msg.steer = steer * steer_ratio_;
        
        publisher_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "Sent motion cmd: throttle %f steer %f from node %s",
        msg.throttle, msg.steer, this->get_name());
    }

    private:

    float throttle_ratio_;
    float steer_ratio_;

    rclcpp::Publisher<teacar_msgs::msg::Motioncmd>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        for(const auto & param : parameters)
        {
            if(param.get_name() == "throttle_ratio")
            {
                throttle_ratio_ = param.as_double();
                RCLCPP_DEBUG(this->get_logger(),"Updated throttle_ratio: %f", throttle_ratio_ );
            }
            else if( param.get_name() == "steer_ratio")
            {
                steer_ratio_ = param.as_double();
                RCLCPP_DEBUG(this->get_logger(), "Updated steer_ratio: %f", steer_ratio_);
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters updated successfully";
        return result;
    }

};

}







#endif  // CONTROLLER__CONTROLLER_HPP_