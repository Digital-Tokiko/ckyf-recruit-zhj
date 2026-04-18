//
// Created by mizu on 2026/4/16.
//

#include "Predictor.h"

void Predictor::CalcFire() {
    bool target_found = false;
    float angle;
    double y;
	double x;
    std::map<double, MyKalmanFilter*> temp_filters;
    std::map<double, MyKalmanFilter*> temp_friend_filters;

    if (type_ == RED) {
        for (auto &it : filters_[BLUE]) {
            temp_filters[it.first] = it.second.get();
        }
        for (auto &it : filters_[RED]) {
            temp_friend_filters[it.first] = it.second.get();
        }
        for (auto &it : filters_[GREY]) {
            temp_friend_filters[it.first] = it.second.get();
        }
    }
    else if (type_ == BLUE) {
        for (auto &it : filters_[RED]) {
            temp_filters[it.first] = it.second.get();
        }
        for (auto &it : filters_[BLUE]) {
            temp_friend_filters[it.first] = it.second.get();
        }
        for (auto &it : filters_[GREY]) {
            temp_friend_filters[it.first] = it.second.get();
        }
    }

    if (temp_filters.empty()) return;
    auto it = temp_filters.begin();

    while (!target_found) {
        if (temp_filters.empty()) return;

        if (!last_avoid_.empty()) {
            for (auto it2 = last_avoid_.begin(); it2 != last_avoid_.end(); ++it2) {
                it = temp_filters.find(*it2);
                if (it != temp_filters.end()) {
                    last_avoid_.erase(it2);
                    break;
                }
            }
        } else it = temp_filters.find(last_aim_);
        /*if (it == temp_filters.end()) {
            for (auto it1: target_health_) {
                if (it1.second == min_health_) {
                    it = temp_filters.find(it1.first);
                }
            }
            if (it == temp_filters.end()) it = temp_filters.find(target_index_.begin()->second);
            if (it == temp_filters.end()) it = std::prev(temp_filters.end());
        }*/
        if (it == temp_filters.end()) it = std::prev(temp_filters.end());

        /*if (it->second.GetFilterCnt() < 18 || fabs(it->second.GetXk().at<double>(1, 0)) < 80) {
            temp_filters.erase(it->first);
            continue;
        }*/

        cv::Mat Xk = it->second->GetXk();
        x = Xk.at<double>(0, 0);
        double v = Xk.at<double>(1, 0);
		double a = Xk.at<double>(2, 0);

        y = it->first;

        cv::Point point_predicted(x /* + 32 */, y);


		double time_gap = cv::norm(point_predicted - fort_point_) / 600;

        CalcX(point_predicted, v ,a,time_gap);


        angle = atan2(fort_point_.y - y, point_predicted.x - fort_point_.x) * 180 / M_PI; //dy取了相反数

        if (temp_friend_filters.empty()) {
            target_found = true;
            break;
        } else {
            auto it1 = temp_friend_filters.begin();
            for (; it1 != temp_friend_filters.end(); ++it1) {
                float angle1;
				float angle2;
                cv::Mat Xk1 = it1->second->GetXk();
                double x1 = Xk1.at<double>(0, 0);
                double v1 = Xk1.at<double>(1, 0);
				double a1 = Xk1.at<double>(2, 0);
                double y1 = it1->first;

                cv::Point point_predicted1(x1 /* + 32 */, y1);

				double time_gap1 = cv::norm(point_predicted1 - fort_point_) / 600;

                CalcX(point_predicted1, v1 , a1,time_gap1);


                angle1 = atan2(fort_point_.y - y1, point_predicted1.x - fort_point_.x + 32) * 180 / M_PI; //dy取了相反数
				angle2 = atan2(fort_point_.y - y1, point_predicted1.x - fort_point_.x - 32) * 180 / M_PI; //dy取了相反数

				//ToDraw.push_back( cv::Rect (cv::Point(point_predicted1.x- 5,y1-5), cv::Point(point_predicted1.x+5,y1+5)));

				if (angle1 < angle && angle2 > angle) break;
            }
            if (it1 == temp_friend_filters.end()) { target_found = true; } else {
                last_avoid_.push_back(y);
                temp_filters.erase(y);
            }
        }
    }

    std_msgs::msg::Float64 angle_msg;
    angle_msg.data=angle;
    angle_publisher_->publish(angle_msg);
    last_aim_ = y;
}

void Predictor::CalcX(cv::Point& point,double v,double a,double init){
    double low =0,high = init * 2,mid = (high+low)/2;
    double init_x=point.x;
    const static int max_try = 50;
    for (int i = 0; i < max_try; i++) {
        mid = (low + high)/2;
        if (fabs(high - low) < 1e-4f) break;

        point.x = init_x +  v * mid + 0.5 * mid * mid * a ;
        if ( cv::norm(point - fort_point_)/600 > mid) {
            low = mid;
        }
        else if (cv::norm(point - fort_point_)/600 < mid){
            high = mid;
        }
    }
    point.x = init_x +  v * mid + 0.5 * mid * mid * a ;

    //std::cout << mid << std::endl;
}

void Predictor::BoardCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(filters_mutex_);

    double y=-1;
    //因为用y坐标来区分目标，所以需要化归来防止被错误区分开
    for (auto &it: ys_) {
        if (fabs(it - msg->point.y) < 7) {
            y = it;
            break;
        }
    }

    double dt;

    if (y<0) {
        y = msg->point.y;
        ys_.push_back(y);

        if (last_time_[RED].find(y)!=last_time_[RED].end()) {
            last_time_[RED][y]= msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        }
        else if (last_time_[BLUE].find(y)!=last_time_[BLUE].end()) {
            last_time_[BLUE][y]= msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        }
        else last_time_[msg->point.z][y] = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

        if (msg->point.x < 100) filters_[msg->point.z].insert({y,std::make_unique<MyKalmanFilter>(ca_left_)});
        else filters_[msg->point.z].insert({y,std::make_unique<MyKalmanFilter>(ca_right_)});

        double init_dt = 0.016;
        cv::Mat A = makeA(init_dt);
        filters_[msg->point.z][y]->Predict(A, makeQ(init_dt));
        filters_[msg->point.z][y]->Update((cv::Mat_<double>(1, 1) << msg->point.x));
    }

    else{
        double sec = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        dt = sec - last_time_[msg->point.z][y];
        cv::Mat A = makeA(dt);
        last_time_[msg->point.z][y] = sec;
        if (filters_[msg->point.z].find(y)!=filters_[msg->point.z].end()) {
            filters_[msg->point.z].find(y)->second->Predict(A,makeQ(dt));
            filters_[msg->point.z].find(y)->second->Update((cv::Mat_<double>(1, 1) << msg->point.x));
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Predictor)