#pragma once

#include <memory>

#include <Eigen/Eigen>

namespace SINS {

enum class SensorType {
    kUnknown,
    kIMU,
    kWheel,
    kGNSS
};

struct SensorData {
    using Ptr = std::shared_ptr<SensorData>;
    using ConstPtr = std::shared_ptr<const SensorData>;

    SensorData(double _time, const SensorType &_type, const std::string &_name) :
        time(_time), type(_type), name(_name) { }
    
    virtual ~SensorData() { }

    double time;
    SensorType type;
    std::string name;
};

struct ImuData : public SensorData {
    using Ptr = std::shared_ptr<ImuData>;
    using ConstPtr = std::shared_ptr<const ImuData>;

    enum Mode {
        kRate,
        kIncreasement,
    };

    ImuData() : SensorData(-1.0, SensorType::kIMU, ""), mode(Mode::kIncreasement) { }
    ~ImuData() override { }

    Mode mode;
    Eigen::Vector3d acc;   // acclearation [m/s^2] or velocity increasement [m/s]
    Eigen::Vector3d gyro;  // angular rate [rad/s] or angle increasement [rad]
};

struct WheelData : public SensorData {
    using Ptr = std::shared_ptr<WheelData>;
    using ConstPtr = std::shared_ptr<const WheelData>;

    WheelData() : SensorData(-1.0, SensorType::kWheel, "") { }

    double left_rear_vel;
    double right_rear_vel;
};

struct GnssData : public SensorData {
    using Ptr = std::shared_ptr<GnssData>;
    using ConstPtr = std::shared_ptr<const GnssData>;

    GnssData() : SensorData(-1.0, SensorType::kGNSS, "") { }

    int pos_type;
    Eigen::Vector3d lat_lon_hei;
    Eigen::Vector3d velocity;

    Eigen::Vector3d lat_lon_hei_std;
    Eigen::Vector3d velocity_std;
};

}  // namespace SINS