//
// Created by mizu on 2026/4/18.
//

#ifndef RECRUIT_DRAWER_H
#define RECRUIT_DRAWER_H

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp"

enum type {
    RED,
    BLUE,
    GREEN,
};

struct RectWithColor {
    cv::Rect rect;
    uint8_t color;
};

class Drawer : public rclcpp::Node{
    private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr draw_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_subscriber_;

    cv_bridge::CvImagePtr raw_img_;
    std::vector<RectWithColor> ToDraw_;
    cv::Mat raw_img_mat_;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void drawerCallback(const geometry_msgs::msg::Point::ConstSharedPtr &msg);
    public:
    explicit Drawer(const rclcpp::NodeOptions& options) : rclcpp::Node("drawer",options) {
        raw_img_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&Drawer::imageCallback, this, std::placeholders::_1));

        draw_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "draw", 10, std::bind(&Drawer::drawerCallback, this, std::placeholders::_1));
    }
};


#endif //RECRUIT_DRAWER_H