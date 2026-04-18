//
// Created by mizu on 2026/2/22.
//

#include "RawReceiver.h"
#include <cmath>

void RawReceiver::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

    raw_img_ = cv_bridge::toCvCopy(msg);

    ProcessInit();

    if (type != NONE) cv::rectangle(grey_img_mat_, cv::Point(0, raw_img_mat_.rows - 100),
                                    cv::Point(raw_img_mat_.cols, raw_img_mat_.rows), cv::Scalar(0), -1);
    cv::findContours(grey_img_mat_, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //过滤掉背景数字造成的杂音
    for (auto &it: contours_) if (cv::boundingRect(it).area() > 150) contours_rect_.push_back(cv::boundingRect(it));

    bg_mask_ = cv::Mat::zeros(raw_img_mat_.size(), CV_8UC1);

    //cv::Mat test_mask_ = cv::Mat::zeros(raw_img_mat_.size() , CV_8UC3);

    cv::Mat hsv;
    cv::cvtColor(raw_img_mat_, hsv, cv::COLOR_BGR2HSV);

    if (type != NONE)
        for (auto &it: contours_rect_) {
            //遍历场上目标进行处理
            if (it.area() < 1000) continue;
            it.x += 1;
            it.y += 3;
            it.width -= 1;
            it.height -= 4;

            bg_mask_ = cv::Mat::zeros(raw_img_mat_.size(), CV_8UC1);
            cv::rectangle(bg_mask_, it, cv::Scalar(255), -1);


            double h_temp = cv::mean(hsv, bg_mask_)[0];
            double s_temp = cv::mean(hsv, bg_mask_)[1];

            double x_mid = it.x + it.width/2;

            geometry_msgs::msg::PointStamped board_bottom_mid_point;
            board_bottom_mid_point.header.stamp = this->now();

            if (x_mid >= 1102 || x_mid <=50) break;

            geometry_msgs::msg::Point to_draw;

            to_draw.x = x_mid;
            to_draw.y = it.y + it.height;
            to_draw.z = 0;

            draw_publisher_ -> publish(to_draw);


            board_bottom_mid_point.point.x = x_mid;
            board_bottom_mid_point.point.y = it.y + it.height;

            if (h_temp > 4) {
                //蓝色目标
                board_bottom_mid_point.point.z = 1;
            } else if (s_temp > 1) {
                //红色目标
                board_bottom_mid_point.point.z = 0;
            }
            else {
                //所有灰色目标
                //打中后的目标变灰，防止丢失
                board_bottom_mid_point.point.z = 2;
            }

            board_publisher_->publish(board_bottom_mid_point);
        }

    if (type == NONE) {
        //初始化识别炮塔颜色
        int max_area_index = 0;
        for (int i = 0; i < contours_rect_.size(); i++) if (
            contours_rect_[i].area() > contours_rect_[max_area_index].area()) max_area_index = i;

        contours_rect_[max_area_index].x += 15;
        contours_rect_[max_area_index].y += 15;
        contours_rect_[max_area_index].width /= 2;
        contours_rect_[max_area_index].height /= 2;
        cv::rectangle(bg_mask_, contours_rect_[max_area_index], cv::Scalar(255), -1);

        geometry_msgs::msg::Point fort_point_msg;
        fort_point_msg.x = contours_rect_[max_area_index].x + contours_rect_[max_area_index].width / 2;
        fort_point_msg.y = contours_rect_[max_area_index].y + contours_rect_[max_area_index].height / 2;
        fort_point_msg.z = 0;



        fort_point_publisher_->publish(fort_point_msg);

        //cv::rectangle(test_mask_ , contours_rect_[max_area_index] , cv::Scalar(255,255,255) , -1);

        double h_temp = cv::mean(hsv, bg_mask_)[0];
        if (h_temp <= 10) {
            type = RED;
        } else type = BLUE;

        type_msg_.data = type;
        type_publisher_->publish(type_msg_);

/*       if (type == RED) {
            std::cout << "Red" << std::endl;
        }
        else {
            std::cout << "Blue" << std::endl;
        }*/
    }

    else {
        for ( auto &it : contours_rect_ ) {
            if (it.area() < 1000) continue;
            //cv::rectangle(test_mask_ , it, cv::Scalar(255,255,255) , -1);
        }
    }

    /*cv::bitwise_and(raw_img_mat_ , test_mask_, raw_img_mat_);

    cv::imshow("raw_img", raw_img_mat_);
    cv::waitKey(1);*/
    //cv::destroyWindow("raw_img");


}

void RawReceiver::ProcessInit() {  //初始化操作
    raw_img_mat_ = raw_img_->image;

    cv::cvtColor(raw_img_mat_, raw_img_mat_, cv::COLOR_RGB2BGR);
    cv::cvtColor(raw_img_mat_, grey_img_mat_, cv::COLOR_BGR2GRAY);

    contours_.clear();
    contours_rect_.clear();

    cv::threshold(grey_img_mat_, grey_img_mat_, 255, 0, 255);
    cv::GaussianBlur(grey_img_mat_, grey_img_mat_, cv::Size(3, 3), 1.0);
    cv::Canny(grey_img_mat_, grey_img_mat_, 100, 200, 3);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RawReceiver)