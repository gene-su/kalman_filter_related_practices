#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {
    is_initialized_ = false;
    std_px_ = 0.15;
    std_py_ = 0.15;
    std_ax_ = 5.;
    std_ay_ = 5.;
}

void KalmanFilter::ProcessMeasurement(const Measurement& measurement) {
    if (!is_initialized_) {
        if (measurement.sensor_type == Measurement::SensorType::Lidar) {
            x = Eigen::Vector4d::Zero();
            x(0) = measurement.raw_data(0);
            x(1) = measurement.raw_data(1);
            P = Eigen::Matrix4d::Identity();
            P(0, 0) = pow(std_px_, 2);
            P(1, 1) = pow(std_py_, 2);
        }

        previous_timestamp_ = measurement.timestamp;
        is_initialized_ = true;
        return;
    }

    double delta_t = (measurement.timestamp - previous_timestamp_) / 1e6;
    previous_timestamp_ = measurement.timestamp;

    Predict(delta_t);
    Update(measurement);
    std::cout << "x = " << std::endl;
    std::cout << x << std::endl;
    std::cout << "P = " << std::endl;
    std::cout << P << std::endl;
}

void KalmanFilter::Predict(const double delta_t) {
    // transition matrix
    Eigen::Matrix4d F;
    F << 1., 0., delta_t, 0., 0., 1., 0., delta_t, 0., 0., 1., 0., 0., 0., 0.,
        1.;
    // process covariance matrix
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = pow(delta_t, 4) / 4 * std_ax_;
    Q(0, 2) = pow(delta_t, 3) / 2 * std_ax_;
    Q(1, 1) = pow(delta_t, 4) / 4 * std_ay_;
    Q(1, 3) = pow(delta_t, 3) / 2 * std_ay_;
    Q(2, 0) = pow(delta_t, 3) / 2 * std_ax_;
    Q(2, 2) = pow(delta_t, 2) * std_ax_;
    Q(3, 1) = pow(delta_t, 3) / 2 * std_ay_;
    Q(3, 3) = pow(delta_t, 2) * std_ay_;

    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::Update(const Measurement& measurement) {
    // measurement matrix
    Eigen::MatrixXd H(2, 4);
    H << 1., 0., 0., 0., 0., 1., 0., 0.;
    // measurement covariance matrix
    Eigen::MatrixXd R(2, 2);
    R << 0.0225, 0., 0., 0.0225;

    Eigen::VectorXd y = measurement.raw_data - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
}