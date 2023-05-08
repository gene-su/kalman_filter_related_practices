#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include "Eigen/Dense"
#include "kalman_filter.h"

#include "measurement.h"

void LoadMeasurements(std::vector<Measurement>& measurements) {
    measurements.clear();

    std::ifstream file("../data/obj_pose-laser-radar-synthetic-input.txt");
    if (!file.is_open()) {
        std::cout << "Cannot open file" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream line_stream(line);

        std::string sensor_type;
        line_stream >> sensor_type;
        if (sensor_type == "L") {
            double x, y;
            int64_t timestamp;
            line_stream >> x;
            line_stream >> y;
            line_stream >> timestamp;

            Measurement measurement;
            measurement.sensor_type = Measurement::Lidar;
            measurement.raw_data = Eigen::Vector2d(x, y);;
            measurement.timestamp = timestamp;
            measurements.push_back(measurement);
        } else if (sensor_type == "R") {
            // skip radar measurement
            continue;
        }
    }
    if (file.is_open()) {
        file.close();
    }
}

int main() {
    // load measurements
    std::vector<Measurement> measurements;
    LoadMeasurements(measurements);

    KalmanFilter kalman_filter;
    for (const auto& measurement: measurements){
        kalman_filter.ProcessMeasurement(measurement);
    }

    return 0;
}