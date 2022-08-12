// Copyright 2021 Sui Fang

#pragma once

#include <vector>
#include <memory>
#include <sstream>
#include <Eigen/Eigen>
#include <iostream>

namespace kit {
namespace perception {
namespace fusion {

typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct Point3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct BBox2D {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

struct BBox3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    float length = 0.0f;
};

struct RadarObject {
    double time_ns = 0.0f;
    int id = -1;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.5f;
    float velo_x = 0.0f;
    float velo_y = 0.0f;
    float velo_z = 0.0f;
    float angle = 0.0f;
    float rcs = 0.0f;

    std::string ToString() const {
        std::ostringstream oss;
        oss << "time_ns: " << time_ns << ", id: " << id
            << ", (x, y, z)=(" << x << ", " << y << ", " << z << ")"
            << ", (vx, vy, vz)=(" << velo_x << ", " << velo_y << ", " << velo_z << ")"
            << ", angle=" << angle << ", rcs=" << rcs;
        return oss.str();
    }
};
using RadarObjectPtr = std::shared_ptr<RadarObject>;
using RadarObjectConstPtr = std::shared_ptr<const RadarObject>;

struct RadarObjectList {
    double time_ns = 0.0f;
    std::string frame_id = "";
    std::vector<RadarObjectPtr> objs;

    void Reset() {
        time_ns = 0.0f;
        frame_id = "";
        objs.clear();
    }
};
using RadarObjectListPtr = std::shared_ptr<RadarObjectList>;
using RadarObjectListConstPtr = std::shared_ptr<const RadarObjectList>;

struct CameraObject {
    double time_ns = 0.0f;
    int id = -1;
    float ux = 0.0;
    float vy = 0.0;
    float width = 0.0f;
    float height = 0.0f;
    int label = -1;
    float confidence = 0.0f;

    std::string ToString() const {
        std::ostringstream oss;
        oss << "time_ns: " << time_ns << ", id: " << id << ", bbox_2d=("
            << ux << ", " << vy << ", " << width << ", " << height
            << "), label=" << label << ", confidence=" << confidence;
        return oss.str();
    }
};
using CameraObjectPtr = std::shared_ptr<CameraObject>;
using CameraObjectConstPtr = std::shared_ptr<const CameraObject>;

struct CameraObjectList {
    double time_ns = 0.0f;
    std::string frame_id = "";
    std::vector<CameraObjectPtr> objs;

    void Reset() {
        time_ns = 0.0f;
        frame_id = "";
        objs.clear();
    }
};
using CameraObjectListPtr = std::shared_ptr<CameraObjectList>;
using CameraObjectListConstPtr = std::shared_ptr<const CameraObjectList>;

struct LiDARObject {
    double time_ns = 0.0f;
    int id = -1;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    float velo_x = 0.0f;
    float velo_y = 0.0f;
    float velo_z = 0.0f;
    int label = -1;
    float confidence = 0.0f;

    std::string ToString() const {
        std::ostringstream oss;
        oss << "time_ns: " << time_ns << ", id: " << id << ", (x, y, z)=(" << x
            << ", " << y << ", " << z << "), (vx, vy, vz)=(" << velo_x << ", "
            << velo_y << ", " << velo_z << "), (label, confidence)=(" << label
            << ", " << confidence << "), bbox_3d=(" << length << ", " << width
            << ", " << height << ")";
        return oss.str();
    }
};
using LiDARObjectPtr = std::shared_ptr<LiDARObject>;
using LiDARObjectConstPtr = std::shared_ptr<const LiDARObject>;

struct LiDARObjectList {
    double time_ns = 0.0f;
    std::string frame_id = "";
    std::vector<LiDARObjectPtr> objs;

    void Reset() {
        time_ns = 0.0f;
        frame_id = "";
        objs.clear();
    }
};
using LiDARObjectListPtr = std::shared_ptr<LiDARObjectList>;
using LiDARObjectListConstPtr = std::shared_ptr<const LiDARObjectList>;

struct FusionObject {
    double time_ns = 0.0f;
    int id = -1;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    float velo_x = 0.0f;
    float velo_y = 0.0f;
    float velo_z = 0.0f;
    int label = -1;
    float confidence = 0.0f;
    float ux = 0.0f;
    float vy = 0.0f;
    float width_2d = 0.0f;
    float height_2d = 0.0f;
    int life_duration = 0;
    float rcs = 0.0f;

    void transformBy(const Eigen::Affine3d& trans) {
        Eigen::Vector3d pos(x, y, z); Eigen::Vector3d velo(velo_x, velo_y, velo_z);
        pos = trans * pos;
        velo = trans.linear() * velo;
        x = pos(0);
        y = pos(1);
        z = pos(2);
        velo_x = velo(0);
        velo_y = velo(1);
        velo_z = velo(2);
    }

    std::string ToString() const {
        std::ostringstream oss;
        oss << "time_ns: " << time_ns << ", id: " << id << ", (x, y, z)=(" << x
            << ", " << y << ", " << z << "), (vx, vy, vz)=(" << velo_x << ", "
            << velo_y << ", " << velo_z << "), (label, confidence)=(" << label
            << ", " << confidence << "), bbox_3d=(" << length << ", " << width
            << ", " << height << "), bbox_2d=(" << ux << ", " << vy << ", "
            << width_2d << ", " << height_2d << "), life=" << life_duration
            << ", rcs=" << rcs;
        return oss.str();
    }
};
using FusionObjectPtr = std::shared_ptr<FusionObject>;
using FusionObjectConstPtr = std::shared_ptr<const FusionObject>;

struct FusionObjectList {
    double time_ns = 0.0f;
    std::string frame_id = "";
    std::vector<FusionObjectPtr> objs;

    void Reset() {
        time_ns = 0.0f;
        frame_id = "";
        objs.clear();
    }
};
using FusionObjectListPtr = std::shared_ptr<FusionObjectList>;
using FusionObjectListConstPtr = std::shared_ptr<const FusionObjectList>;

struct Frame {
    double time_ns = 0.0f;

    LiDARObjectListPtr lidar_objs;
    CameraObjectListPtr camera_objs;
    RadarObjectListPtr radar_objs;
};
using FramePtr = std::shared_ptr<Frame>;

}  // namespace fusion
}  // namespace perception
}  // namespace kit
