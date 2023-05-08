#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <iostream>

#include "Eigen/Dense"
#include "measurement.h"

class KalmanFilter {
  public:
    KalmanFilter();

    void ProcessMeasurement(const Measurement& measurement);
    void Predict(const double delta_t);
    void Update(const Measurement& measurement);

    Eigen::Vector4d x;
    Eigen::Matrix4d P;

  private:
    bool is_initialized_;
    int64_t previous_timestamp_;

    double std_px_;
    double std_py_;
    double std_ax_;
    double std_ay_;
};

#endif  // KALMAN_FILTER_H_