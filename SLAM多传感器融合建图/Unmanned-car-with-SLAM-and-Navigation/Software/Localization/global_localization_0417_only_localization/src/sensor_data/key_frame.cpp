#include "global_localization/sensor_data/key_frame.hpp"

namespace global_localization {

Eigen::Quaternionf KeyFrame::GetQuaternion() const {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}

Eigen::Vector3f KeyFrame::GetTranslation() const {
    Eigen::Vector3f t = pose.block<3,1>(0,3);

    return t;
}

}