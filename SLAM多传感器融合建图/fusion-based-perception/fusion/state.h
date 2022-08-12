// Copyright 2021 Sui Fang

#pragma once

#include <Eigen/Dense>

namespace kit {
namespace perception {
namespace fusion {

enum SensorType {
    LIDAR,
    CAMERA,
    RADAR
};

struct Measurement {
    double time_ns = 0.0f;

    SensorType sensor_type;
    Eigen::VectorXd meas;
};

struct State {
    double time_ns = 0.0f;

    Eigen::VectorXd meas;
};

}  // namespace fusion
}  // namespace perception
}  // namespace kit
