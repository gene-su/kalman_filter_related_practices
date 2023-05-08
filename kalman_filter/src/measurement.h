#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "Eigen/Dense"

class Measurement {
  public:
    enum SensorType { Lidar, Radar } sensor_type;
    Eigen::VectorXd raw_data;
    int64_t timestamp;
};

#endif /* MEASUREMENT_H_ */