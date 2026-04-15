#ifndef TRACKED_OBSTACLE_H
#define TRACKED_OBSTACLE_H

#include "cbf_manager/kalman.h"

// estimated values
struct obstacle{
    // position
    Eigen::Vector3d pos_;
    // velocity
    Eigen::Vector3d vel_;
};

class TrackedObstacle {
public:
    TrackedObstacle(const Eigen::Vector3d& obstacle_pos) : kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_z_(0, 1, 2) 
    {
        fade_counter_ = s_fade_counter_size_;
        obstacle_.pos_ = obstacle_pos;
        obstacle_.vel_ = Eigen::Vector3d::Zero();
        initKF();
    }

    void predictState() {
        kf_x_.predictState();
        kf_y_.predictState();
        kf_z_.predictState();

        obstacle_.pos_(0) = kf_x_.q_pred(0);
        obstacle_.pos_(1) = kf_y_.q_pred(0);
        obstacle_.pos_(2) = kf_z_.q_pred(0);

        obstacle_.vel_(0) = kf_x_.q_pred(1);
        obstacle_.vel_(1) = kf_y_.q_pred(1);
        obstacle_.vel_(2) = kf_z_.q_pred(1);

        fade_counter_--;
    }

    void correctState(const Eigen::Vector3d& obstacle_pos) {
        kf_x_.y(0) = obstacle_pos(0);
        kf_y_.y(0) = obstacle_pos(1);
        kf_z_.y(0) = obstacle_pos(2);

        kf_x_.correctState();
        kf_y_.correctState();
        kf_z_.correctState();

        obstacle_.pos_(0) = kf_x_.q_est(0);
        obstacle_.pos_(1) = kf_y_.q_est(0);
        obstacle_.pos_(2) = kf_z_.q_est(0);

        obstacle_.vel_(0) = kf_x_.q_est(1);
        obstacle_.vel_(1) = kf_y_.q_est(1);
        obstacle_.vel_(2) = kf_z_.q_est(1);

        fade_counter_ = s_fade_counter_size_;
    }

    void updateState() {
        kf_x_.predictState();
        kf_y_.predictState();
        kf_z_.predictState();

        kf_x_.correctState();
        kf_y_.correctState();
        kf_z_.correctState();

        obstacle_.pos_(0) = kf_x_.q_est(0);
        obstacle_.pos_(1) = kf_y_.q_est(0);
        obstacle_.pos_(2) = kf_z_.q_est(0);

        obstacle_.vel_(0) = kf_x_.q_est(1);
        obstacle_.vel_(1) = kf_y_.q_est(1);
        obstacle_.vel_(2) = kf_z_.q_est(1);

        fade_counter_--;
    }

    static void setSamplingTime(double tp) {
        s_sampling_time_ = tp;
    }

    static void setCounterSize(int size) {
        s_fade_counter_size_ = size;
    }

    static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
        s_process_variance_ = process_var;
        s_process_rate_variance_ = process_rate_var;
        s_measurement_variance_ = measurement_var;
    }

    bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
    const obstacle& getObstacle() const { return obstacle_; }
    const KalmanFilter& getKFx() const { return kf_x_; }
    const KalmanFilter& getKFy() const { return kf_y_; }
    const KalmanFilter& getKFr() const { return kf_z_; }

private:
    obstacle obstacle_;

    KalmanFilter kf_x_;
    KalmanFilter kf_y_;
    KalmanFilter kf_z_;

    int fade_counter_;

    // common variables
    static int s_fade_counter_size_;
    static double s_sampling_time_;
    static double s_process_variance_;
    static double s_process_rate_variance_;
    static double s_measurement_variance_;

    void initKF() {
        kf_x_.A(0, 1) = s_sampling_time_;
        kf_y_.A(0, 1) = s_sampling_time_;
        kf_z_.A(0, 1) = s_sampling_time_;

        kf_x_.C(0, 0) = 1.0;
        kf_y_.C(0, 0) = 1.0;
        kf_z_.C(0, 0) = 1.0;

        kf_x_.R(0, 0) = s_measurement_variance_;
        kf_y_.R(0, 0) = s_measurement_variance_;
        kf_z_.R(0, 0) = s_measurement_variance_;

        kf_x_.Q(0, 0) = s_process_variance_;
        kf_y_.Q(0, 0) = s_process_variance_;
        kf_z_.Q(0, 0) = s_process_variance_;

        kf_x_.Q(1, 1) = s_process_rate_variance_;
        kf_y_.Q(1, 1) = s_process_rate_variance_;
        kf_z_.Q(1, 1) = s_process_rate_variance_;

        kf_x_.q_pred(0) = obstacle_.pos_(0);
        kf_y_.q_pred(0) = obstacle_.pos_(1);
        kf_z_.q_pred(0) = obstacle_.pos_(2);

        kf_x_.q_pred(1) = obstacle_.vel_(0);
        kf_y_.q_pred(1) = obstacle_.vel_(1);
        kf_z_.q_pred(1) = obstacle_.vel_(2);

        kf_x_.q_est(0) = obstacle_.pos_(0);
        kf_y_.q_est(0) = obstacle_.pos_(1);
        kf_z_.q_est(0) = obstacle_.pos_(2);

        kf_x_.q_est(1) = obstacle_.vel_(0);
        kf_y_.q_est(1) = obstacle_.vel_(1);
        kf_z_.q_est(1) = obstacle_.vel_(2);

        kf_x_.y(0) = obstacle_.pos_(0);
        kf_y_.y(0) = obstacle_.pos_(1);
        kf_z_.y(0) = obstacle_.pos_(2);
    }
};

#endif // TRACKED_OBSTACLE_H