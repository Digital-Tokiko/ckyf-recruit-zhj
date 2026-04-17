//
// Created by mizu on 2026/2/22.
//

#ifndef AUTOAIM_WS_RAWRECEIVER_H
#define AUTOAIM_WS_RAWRECEIVER_H

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/bool.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

enum type {
    RED,
    BLUE,
    NONE
};

class RawReceiver : public rclcpp::Node {
private:
	std::vector<cv::Rect> ToDraw;

	double last_time_;
	double dt_;

    std_msgs::msg::Bool type_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr type_publisher_; //假红真蓝
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr fort_point_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr board_publisher_; //x，y为原意，z表示种类，0红1蓝2灰
    cv_bridge::CvImagePtr raw_img_;

    cv::Mat raw_img_mat_;
    cv::Mat grey_img_mat_;
    cv::Mat bg_mask_;

    std::vector<std::vector<cv::Point> > contours_;
    std::vector<cv::Rect> contours_rect_;

    uint8_t type; //2代表未准备，0代表红色，1代表蓝色

    void ProcessInit();

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
public:
    explicit RawReceiver(const rclcpp::NodeOptions& options) : rclcpp::Node("raw_receiver",options) {

        raw_img_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&RawReceiver::imageCallback, this, std::placeholders::_1));

        type_publisher_ = this->create_publisher<std_msgs::msg::Bool>("self_type", 10);

        fort_point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("fort_point",10);

        board_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("board_state", 10);

        type = NONE;
    }

    ~RawReceiver() = default;
};


#endif //AUTOAIM_WS_RAWRECEIVER_H
