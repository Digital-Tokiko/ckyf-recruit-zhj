//
// Created by mizu on 2026/4/16.
//

#ifndef RECRUIT_PREDICTOR_H
#define RECRUIT_PREDICTOR_H

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "include/MyKalmanFilter/MyKalmanFilter.h"

#include <iostream>

class Predictor :public rclcpp::Node {
private:
    cv::Point fort_point_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr fort_point_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr board_subscriber_;

    void FortPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        fort_point_.x = msg->x;
        fort_point_.y = msg->y;
        std::cout << fort_point_.x << " " << fort_point_.y << std::endl;
    }

public:
    explicit Predictor(const rclcpp::NodeOptions& options) : rclcpp::Node("predictor",options) {
        fort_point_ = cv::Point(0,0);
        fort_point_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("fort_point", 10, std::bind(&Predictor::FortPointCallback, this, std::placeholders::_1));
        board_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("board_state",10, std::bind(&Predictor::BoardCallback, this, std::placeholders::_1));
    }
};


#endif //RECRUIT_PREDICTOR_H