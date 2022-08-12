#pragma once

#include <Eigen/Core>

namespace TGK {
namespace Util {

inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;
    return w;
}

}  // namespace Util
}  // namespace TGK