//
// Created by mizu on 2026/2/23.
//

#ifndef AUTOAIM_WS_MYKALMANFILTER_H
#define AUTOAIM_WS_MYKALMANFILTER_H

#include <opencv2/opencv.hpp>

class MyKalmanFilter {
private:
    cv::Mat x_k_;
    cv::Mat x_k_1_; //先前的状态
    cv::Mat K_k_;
    cv::Mat P_k_;
    cv::Mat P_k_1_;

    cv::Mat A_;
    cv::Mat B_;
    cv::Mat H_;
    cv::Mat Q_;
    cv::Mat R_;

    cv::Mat P_k_Expected_;
    cv::Mat x_k_Expected_;

public:
    MyKalmanFilter(const cv::Mat &x0, const cv::Mat &A, const cv::Mat &H, const cv::Mat &P, const cv::Mat &Q,
                   const cv::Mat &R) {
        x_k_1_ = x0;
        this->A_ = A;
        this->H_ = H;
        this->P_k_1_ = P;
        this->Q_ = Q;
        this->R_ = R;
    }

    MyKalmanFilter(const cv::Mat &x0, const cv::Mat &A, const cv::Mat &H, const cv::Mat &P, const cv::Mat &Q,
                   const cv::Mat &R,
                   const cv::Mat &B) {
        x_k_1_ = x0;
        this->A_ = A;
        this->H_ = H;
        this->P_k_1_ = P;
        this->Q_ = Q;
        this->R_ = R;
        this->B_ = B;
    }

    void Predict() {
        x_k_Expected_ = A_ * x_k_1_;
        P_k_Expected_ = A_ * P_k_1_ * A_.t() + Q_;
    }

    void Predict(cv::Mat &u_k_1) {
        x_k_Expected_ = A_ * x_k_1_ + B_ * u_k_1;
        P_k_Expected_ = A_ * P_k_1_ * A_.t() + Q_;
    }

    void Update(cv::Mat &z_k) {
        K_k_ = P_k_Expected_ * H_.t() / (H_ * P_k_Expected_ * H_.t() + R_);
        x_k_ = x_k_Expected_ + K_k_ * (z_k - H_ * x_k_Expected_);
        cv::Mat KH = K_k_ * H_;
        P_k_ = (cv::Mat::eye(KH.rows, KH.cols, CV_64F) - K_k_ * H_) * P_k_Expected_;

        x_k_1_ = x_k_;
        P_k_1_ = P_k_;
    }

    cv::Mat GetXk_Expected() {return x_k_Expected_;}
    cv::Mat GetXk() { return x_k_; }
};


#endif //AUTOAIM_WS_MYKALMANFILTER_H
