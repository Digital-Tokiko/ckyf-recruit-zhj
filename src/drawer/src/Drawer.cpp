//
// Created by mizu on 2026/4/18.
//

#include "Drawer.h"

void Drawer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    raw_img_ = cv_bridge::toCvCopy(msg);
    raw_img_mat_ = raw_img_->image;
    cv::cvtColor(raw_img_mat_, raw_img_mat_, cv::COLOR_RGB2BGR);

    for (auto &it: ToDraw_) {
        if (it.color == RED) cv::rectangle(raw_img_mat_,it.rect, cv::Scalar(0, 0, 255), 2, cv::LINE_8);//0红1蓝2绿 //BGR
        else if (it.color == BLUE) cv::rectangle(raw_img_mat_,it.rect, cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        else cv::rectangle(raw_img_mat_,it.rect, cv::Scalar(0, 255 , 0), 2, cv::LINE_8);
    }
    ToDraw_.clear();


    cv::imshow("visualization", raw_img_mat_);
    cv::waitKey(1);
}

void Drawer::drawerCallback(const geometry_msgs::msg::Point::ConstSharedPtr &msg) {
    RectWithColor temp_rect;
    temp_rect.color = msg->z;
    temp_rect.rect = cv::Rect (cv::Point(msg->x-5,msg->y-5), cv::Point(msg->x+5,msg->y+5));

    ToDraw_.push_back(temp_rect);
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Drawer)