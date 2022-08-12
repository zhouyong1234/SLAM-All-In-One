// Copyright 2021 Sui Fang

#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "object.h"
#include "iou.h"

namespace kit {
namespace perception {
namespace fusion {

// Helper function to convert to string with specified number of decimals
template <typename T>
std::string ToStringWithPrecision(const T a_value, const int n = 2) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

// Helper function to convert hex int to OpenCV RGB (scalar) color
cv::Scalar Hex2RGB(int hexValue);

float DistanceIn3D(const FusionObjectPtr &local, const FusionObjectPtr &global);

void Project3DBoxTo2DBox(
        const BBox3D &box_3d, const Eigen::Affine3d &extrinsic,
        const Vector6d &intrinsic, BBox2D &box_2d);

Eigen::Vector2d ProjectPoint2Image(const Eigen::Vector3d& point,
        const Eigen::Affine3d& extrinsic, const Vector6d& intrinsic);

float IoUIn2D(const BBox2D &box_2d, const BBox3D &box_3d,
        const Eigen::Affine3d& extrinsic, const Vector6d &intrinsic);
float IoUIn2D(const FusionObjectPtr &local, const FusionObjectPtr &global);
float IoUIn2D(const BBox2D &pred, const BBox2D &tgt);

float IoUIn3D(const LiDARObjectPtr& local, const FusionObjectPtr& global);
float IoUIn3D(const BBox3D &pred, const BBox3D& tgt);

}  // namespace fusion
}  // namespace perception
}  // namespace kit
