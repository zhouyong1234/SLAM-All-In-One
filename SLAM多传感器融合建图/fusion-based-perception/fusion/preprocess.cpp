#include "preprocess.h"


namespace kit {
namespace perception {
namespace fusion {

void PreProcess(const LiDARObjectListPtr &lidar_obj_list, Eigen::Affine3d &extrinsic) {
    for (size_t i = 0; i < lidar_obj_list->objs.size(); ++i) {
        const auto& obj = lidar_obj_list->objs[i];
        Eigen::Vector3d pos(obj->x, obj->y, obj->z);
        Eigen::Vector3d velo(obj->velo_x, obj->velo_y, obj->velo_z);
        pos = extrinsic * pos;
        velo = extrinsic.linear() * velo;
        obj->x = pos(0);
        obj->y = pos(1);
        obj->z = pos(2);
        obj->velo_x = velo(0);
        obj->velo_y = velo(1);
        obj->velo_z = velo(2);
    }
}

void PreProcess(const RadarObjectListPtr &radar_obj_list, Eigen::Affine3d &extrinsic) {
    for (size_t i = 0; i < radar_obj_list->objs.size(); ++i) {
        const auto& obj = radar_obj_list->objs[i];
        Eigen::Vector3d pos(obj->x, obj->y, obj->z);
        Eigen::Vector3d velo(obj->velo_x, obj->velo_y, obj->velo_z);
        pos = extrinsic * pos;
        velo = extrinsic.linear() * velo;
        obj->x = pos(0);
        obj->y = pos(1);
        obj->z = pos(2);
        obj->velo_x = velo(0);
        obj->velo_y = velo(1);
        obj->velo_z = velo(2);
    }

}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
