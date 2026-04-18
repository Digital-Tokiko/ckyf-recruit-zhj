//
// Created by mizu on 2026/4/16.
//

#ifndef RECRUIT_PREDICTOR_H
#define RECRUIT_PREDICTOR_H

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "MyKalmanFilter.h"
#include <map>
#include <vector>
#include <memory>
#include <mutex>

#include <iostream>

enum type {
    RED,
    BLUE,
    GREY
};

class Predictor :public rclcpp::Node {
private:
   	cv::Point fort_point_;

	std::vector<double> last_avoid_;

	double last_aim_;

    bool type_; //假红真蓝

	rclcpp::TimerBase::SharedPtr aim_timer_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr fort_point_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr type_subscriber_; //假红真蓝
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr board_subscriber_;

	std::mutex filters_mutex_;
    std::vector<std::map<double, std::unique_ptr<MyKalmanFilter>>> filters_; //0红1蓝2灰
    std::vector<std::map<double,double>> last_time_; //其实可以和滤波器整合为一个数据结构，但是有点小麻烦还是算了（
    std::vector<double> ys_;

    rclcpp::TimerBase::SharedPtr clear_timer_;

    struct MyKalmanInit ca_left_;
    struct MyKalmanInit ca_right_;

    double R_value_;

    double common_speed_;

    cv::Mat P_;
    cv::Mat H_;
    cv::Mat R_;

	double q_;

    cv::Mat x0_L_;
    cv::Mat x0_R_;

    cv::Mat zk_;



    cv::Mat makeA(double dt){
        return (cv::Mat_<double>(3, 3) << 1, dt,dt* dt/ 2,
                                           0, 1, dt,
                                           0, 0, 1);
    };

	cv::Mat makeQ(double dt){
		double Q_x = dt*dt*dt*dt*dt /20;
		double Q_v = dt*dt*dt / 3;
		double Q_a = dt;
		double Q_x_cov = dt*dt*dt*dt / 8;
		double Q_x_coa = dt*dt*dt / 6;
		double Q_v_coa = dt*dt / 2;

        return q_ * (cv::Mat_<double>(3, 3) << Q_x, Q_x_cov,Q_x_coa,
                                           Q_x_cov, Q_v, Q_v_coa,
                                           Q_x_coa, Q_v_coa, Q_a);
    };

    void CalcX(cv::Point& point,double v,double a,double init);
	void CalcFire();

    void ClearDead() {
        double current_time = now().seconds();

		std::lock_guard<std::mutex> lock(filters_mutex_);

        for (int i=0;i<3;i++) {
            for (auto it = filters_[i].begin(); it != filters_[i].end();) {
                //除去不需要的滤波对象***
                if (current_time - last_time_[i][it->first] > 1) {
                    last_time_[i].erase(it->first);
                    for (auto it1=ys_.begin();it1!=ys_.end();it1++) {
                        if (*it1 == it->first) {
                            ys_.erase(it1);
                            break;
                        }
                    }
                    it = filters_[i].erase(it);
                } else ++it;
            }
        }
    }

	void FortPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        fort_point_.x = msg->x;
        fort_point_.y = msg->y;
        std::cout << fort_point_.x << " " << fort_point_.y << std::endl;
    }

    void TypeCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
        type_ = msg->data;

        if (!type_) RCLCPP_INFO(this->get_logger(), "red");
        else RCLCPP_INFO(this->get_logger(), "blue");

        //fort_.TurnTo(90);
    }

    void BoardCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) ;

public:
    explicit Predictor(const rclcpp::NodeOptions& options) : rclcpp::Node("predictor",options) {

        fort_point_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("fort_point", 10, std::bind(&Predictor::FortPointCallback, this, std::placeholders::_1));
        type_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("self_type", 10, std::bind(&Predictor::TypeCallback, this, std::placeholders::_1));

		angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("angle", 10);

        board_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("board_state",10, std::bind(&Predictor::BoardCallback, this, std::placeholders::_1));

        this->declare_parameter("R", 0.03);
        this->declare_parameter("q", 1.0);
        this->declare_parameter("common_speed", 200.0);

        this->get_parameter("R", R_value_);
        this->get_parameter("q", q_);
        this->get_parameter("common_speed", common_speed_);

        P_ = cv::Mat::eye(3, 3, CV_64F);
        H_ = (cv::Mat_<double>(1, 3) << 1, 0, 0);

        R_ = (cv::Mat_<double>(1, 1) << R_value_);

        x0_L_ = (cv::Mat_<double>(3, 1) << 50,
                                              common_speed_,
                                              0);

        x0_R_ = (cv::Mat_<double>(3, 1) << 1102,
                                              -common_speed_,
                                              -0);

        ca_left_ = MyKalmanInit{ P_, H_,  R_, x0_L_};
        ca_right_ = MyKalmanInit{ P_, H_,  R_, x0_R_};

        for (int i=0;i<3;i++) filters_.push_back(std::map<double,std::unique_ptr<MyKalmanFilter>>());
		for (int i=0;i<3;i++) last_time_.push_back(std::map<double,double>());

        zk_ = cv::Mat::zeros(1, 1, CV_64F);

        clear_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Predictor::ClearDead, this));
		aim_timer_ = this->create_wall_timer(std::chrono::milliseconds(80), std::bind(&Predictor::CalcFire, this));

		std_msgs::msg::Float64 angle_msg;
    	angle_msg.data=90;
    	angle_publisher_->publish(angle_msg);

        this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            this->get_parameter("R", R_value_);
            this->get_parameter("q", q_);
            this->get_parameter("common_speed", common_speed_);

            x0_L_ = (cv::Mat_<double>(3, 1) << 50,
                                              common_speed_,
                                              0);

            x0_R_ = (cv::Mat_<double>(3, 1) << 1102,
                                              -common_speed_,
                                              -0);

            R_ = (cv::Mat_<double>(1, 1) << R_value_);
            ca_left_ = MyKalmanInit{ P_, H_,  R_, x0_L_};
            ca_right_ = MyKalmanInit{ P_, H_,  R_, x0_R_};

            for (auto &it1 : filters_) {
                for (auto &it2 : it1) {
                    it2.second->ResetPa();
                    it2.second->ResetR(R_);
                    }
                }
            return result;
            });
    }
};


#endif //RECRUIT_PREDICTOR_H