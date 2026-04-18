//
// Created by mizu on 2026/4/16.
//

#ifndef RECRUIT_DECIDER_H
#define RECRUIT_DECIDER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Fort.h>

class Decider :public rclcpp::Node {
    private:
    std::string port_;

    Fort fort_;

	rclcpp::TimerBase::SharedPtr fire_timer_;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber_;

    void Fire() {
        fort_.Fire();
        fire_timer_->cancel();
    }

    void AimAndFire(const std_msgs::msg::Float64::SharedPtr msg);

    public:

    explicit Decider(const rclcpp::NodeOptions& options) : rclcpp::Node("decider",options) {

        this->declare_parameter("serial", "/dev/pts/0");
        this->get_parameter("serial", port_);

		angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("angle", 10, std::bind(&Decider::AimAndFire, this, std::placeholders::_1));

        fort_=Fort(port_,115200);

    }

};


#endif //RECRUIT_DECIDER_H