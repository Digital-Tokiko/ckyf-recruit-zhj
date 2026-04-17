//
// Created by mizu on 2026/4/16.
//

#ifndef RECRUIT_DECIDER_H
#define RECRUIT_DECIDER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <Fort.h>


class Decider :public rclcpp::Node {
    private:

    std::string port_;

    Fort fort_;

    bool type; //假红真蓝

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr type_subscriber_; //假红真蓝

    void TypeCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
        type = msg->data;

        if (!type) RCLCPP_INFO(this->get_logger(), "red");
        else RCLCPP_INFO(this->get_logger(), "blue");

        fort_.TurnTo(90);
    }

    public:

    explicit Decider(const rclcpp::NodeOptions& options) : rclcpp::Node("decider",options) {
        type_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("self_type", 10, std::bind(&Decider::TypeCallback, this, std::placeholders::_1));
        this->declare_parameter("serial", "/dev/pts/0");
        this->get_parameter("serial", port_);

        fort_=Fort(port_,115200);

    }

};


#endif //RECRUIT_DECIDER_H