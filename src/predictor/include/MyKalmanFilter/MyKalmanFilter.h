//
// Created by mizu on 2026/2/23.
//

#ifndef AUTOAIM_WS_MYKALMANFILTER_H
#define AUTOAIM_WS_MYKALMANFILTER_H

#include <opencv2/opencv.hpp>

struct MyKalmanInit {
    cv::Mat P;
    cv::Mat H;
    //cv::Mat Q;
    cv::Mat R;
    cv::Mat x0;
};

class MyKalmanFilter {
private:
    cv::Mat x_k_;
    cv::Mat x_k_1_; //先前的状态
    cv::Mat K_k_;
    cv::Mat P_k_;
    cv::Mat P_k_1_;

    cv::Mat B_;
    cv::Mat H_;
    cv::Mat R_;

    cv::Mat P_k_Expected_;
    cv::Mat x_k_Expected_;

public:
    MyKalmanFilter() = default;

    MyKalmanFilter(const struct MyKalmanInit &init) {
        x_k_1_ = init.x0;
        this->H_ = init.H;
        this->P_k_1_ = init.P;
        //this->Q_ = init.Q;
        this->R_ = init.R;
    }

    /*MyKalmanFilter(const cv::Mat &x0, const cv::Mat &H, const cv::Mat &P, const cv::Mat &Q,
                   const cv::Mat &R) {
        x_k_1_ = x0;
        this->H_ = H;
        this->P_k_1_ = P;
        this->Q_ = Q;
        this->R_ = R;
    }

    MyKalmanFilter(const cv::Mat &x0, const cv::Mat &H, const cv::Mat &P, const cv::Mat &Q,
                   const cv::Mat &R,
                   const cv::Mat &B) {
        x_k_1_ = x0;
        this->H_ = H;
        this->P_k_1_ = P;
        this->Q_ = Q;
        this->R_ = R;
        this->B_ = B;
    }*/

    void Predict(const cv::Mat& A,const cv::Mat& Q) {
    	if (x_k_1_.empty()) {
        	std::cout << "ERROR: empty matrix detected!" << std::endl;
        	return;
   		 }

        x_k_Expected_ = A * x_k_1_;
        P_k_Expected_ = A * P_k_1_ * A.t() + Q;
    }

    void Predict(cv::Mat &u_k_1,const cv::Mat& A,const cv::Mat& Q) {
        x_k_Expected_ = A * x_k_1_ + B_ * u_k_1;
        P_k_Expected_ = A * P_k_1_ * A.t() + Q;
    }

    void Update(const cv::Mat &z_k) {
        K_k_ = P_k_Expected_ * H_.t() / (H_ * P_k_Expected_ * H_.t() + R_);
        x_k_ = x_k_Expected_ + K_k_ * (z_k - H_ * x_k_Expected_);
        cv::Mat KH = K_k_ * H_;
        P_k_ = (cv::Mat::eye(KH.rows, KH.cols, CV_64F) - K_k_ * H_) * P_k_Expected_;

        x_k_1_ = x_k_;
        P_k_1_ = P_k_;

        //std::cout<<x_k_1_<<" ";

    }

    void ResetPa(){
        P_k_.setTo(0);
        P_k_1_.setTo(0);
        P_k_Expected_.setTo(0);
        x_k_.at<double>(2,0) = 0;
        x_k_1_.at<double>(2,0) = 0;
    }

    void ResetR(cv::Mat &R) {
        R_=R;
    }

    cv::Mat GetXk_Expected() {return x_k_Expected_;}
    cv::Mat GetXk() { return x_k_; }
};


#endif //AUTOAIM_WS_MYKALMANFILTER_H
