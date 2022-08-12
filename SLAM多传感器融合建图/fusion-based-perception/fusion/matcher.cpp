// Copyright 2021 Sui Fang
#include "fusion/matcher.h"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <iostream>
#include "iou.h"

#include <Eigen/Geometry>

namespace kit {
namespace perception {
namespace fusion {

Matcher::Matcher(const std::string& config_file) :
    initialized_(false),
    main_sensor_(SensorType::LIDAR) {
    // hungarian_(std::shared_ptr<Hungarian<int>>(new Hungarian<int>())) {

    YAML::Node config = YAML::LoadFile(config_file);
    const auto camera_intrinsic_vec = config["camera_intrinsics"]["P"].as<std::vector<double>>();
    cam_intrinsic_(0) = camera_intrinsic_vec[0];
    cam_intrinsic_(1) = camera_intrinsic_vec[1];
    cam_intrinsic_(2) = camera_intrinsic_vec[2];
    cam_intrinsic_(3) = camera_intrinsic_vec[3];
    cam_intrinsic_(4) = camera_intrinsic_vec[4];
    cam_intrinsic_(5) = camera_intrinsic_vec[5];

    const auto camera_rotate = config["camera_extrinsics"]["rotation"];
    const auto camera_translation = config["camera_extrinsics"]["translation"];
    {
        Eigen::Translation3d translation(camera_translation["x"].as<double>(), camera_translation["y"].as<double>(), camera_translation["z"].as<double>());
        Eigen::Quaterniond quater(camera_rotate["w"].as<double>(), camera_rotate["x"].as<double>(), camera_rotate["y"].as<double>(), camera_rotate["z"].as<double>());
        extrinsic_camera_to_lidar_ = translation * quater.toRotationMatrix();
    }
    const auto lidar_rotate = config["lidar_extrinsics"]["rotation"];
    const auto lidar_translation = config["lidar_extrinsics"]["translation"];
    {
        Eigen::Translation3d translation(lidar_translation["x"].as<double>(), lidar_translation["y"].as<double>(), lidar_translation["z"].as<double>());
        Eigen::Quaterniond quater(lidar_rotate["w"].as<double>(), lidar_rotate["x"].as<double>(), lidar_rotate["y"].as<double>(), lidar_rotate["z"].as<double>());
        extrinsic_lidar_to_baselink_ = translation * quater.toRotationMatrix();
        extrinsic_camera_to_baselink_ = extrinsic_camera_to_lidar_ * extrinsic_lidar_to_baselink_; 
    }
}

bool Matcher::Match(const RadarObjectListPtr& radar_obj_list,
           const FusionObjectListPtr& global_obj_list,
           std::map<size_t, int>& radar_global_map) {
    radar_global_map.clear();
    for (size_t i = 0; i < radar_obj_list->objs.size(); ++i) {
        radar_global_map[i] = -1;
    }
    std::cout << "radar: local : global = " << radar_obj_list->objs.size() << " : " << global_obj_list->objs.size() << std::endl;
    if (global_obj_list->objs.size() == 0) {
        return true;;
    }

    // Job1: implement association algorithm of cam 2d objects and 3D fusion objects
    std::vector<std::vector<float>> weights;
    for (size_t i = 0; i < radar_obj_list->objs.size(); ++i) {
        std::vector<float> row(global_obj_list->objs.size(), 0);
        weights.push_back(row);
        for (size_t j = 0; j < global_obj_list->objs.size(); ++j) {
            weights[i][j] = IoURadarToFusion(radar_obj_list->objs[i], global_obj_list->objs[j]);   
        }
    }

    std::cout << "radar: " << std::endl;
    for (size_t i = 0; i < weights.size(); ++i) {
        for (size_t j = 0; j < weights[i].size(); ++j) {
            std::cout << " " << weights[i][j];
        }
        std::cout << std::endl;
    }

    auto f = [&](size_t r, size_t c) {return weights[r][c];};
    auto res = munkres_algorithm<float>(radar_obj_list->objs.size(),
            global_obj_list->objs.size(),
            f);
    auto is_matching = [&](const size_t r, const size_t c) {
        const auto ii = std::find_if(begin(res), end(res), [&](const auto& x) {
                    return (x.first == r and x.second == c);
                });
        return ii != end(res);
    };
    for (size_t i = 0; i < radar_obj_list->objs.size(); ++i) {
        for (size_t j = 0; j < global_obj_list->objs.size(); ++j) {
            if (is_matching(i, j)) {
                radar_global_map[i] = j;
                std::cout << "(i, j)=(" << i << ", " << radar_global_map[i] << ")" << std::endl;;
            }
        }
    }

    return true;
}

bool Matcher::Match(const CameraObjectListPtr& cam_obj_list,
           const FusionObjectListPtr& global_obj_list,
           std::map<size_t, int>& cam_global_map) {
    cam_global_map.clear();
    for (size_t i = 0; i < cam_obj_list->objs.size(); ++i) {
        cam_global_map[i] = -1;
    }
    if (global_obj_list->objs.size() == 0) {
        return true;;
    }

    // Job1: implement association algorithm of cam 2d objects and 3D fusion objects
    std::vector<std::vector<float>> weights;
    for (size_t i = 0; i < cam_obj_list->objs.size(); ++i) {
        std::vector<float> row(global_obj_list->objs.size(), 0);
        weights.push_back(row);
        for (size_t j = 0; j < global_obj_list->objs.size(); ++j) {
            weights[i][j] = IoUCamToFusion(cam_obj_list->objs[i], global_obj_list->objs[j]);   
        }
    }

    // std::cout << "camera: " << std::endl;
    // for (size_t i = 0; i < weights.size(); ++i) {
    //     for (size_t j = 0; j < weights[i].size(); ++j) {
    //         std::cout << " " << weights[i][j];
    //     }
    //     std::cout << std::endl;
    // }

    auto f = [&](size_t r, size_t c) {return weights[r][c];};
    auto res = munkres_algorithm<float>(cam_obj_list->objs.size(),
            global_obj_list->objs.size(),
            f);
    auto is_matching = [&](const size_t r, const size_t c) {
        const auto ii = std::find_if(begin(res), end(res), [&](const auto& x) {
                    return (x.first == r and x.second == c);
                });
        return ii != end(res);
    };
    for (size_t i = 0; i < cam_obj_list->objs.size(); ++i) {
        for (size_t j = 0; j < global_obj_list->objs.size(); ++j) {
            if (is_matching(i, j)) {
                cam_global_map[i] = j;
                std::cout << "(i, j)=(" << i << ", " << cam_global_map[i] << ")" << std::endl;;
            }
        }
    }

    return true;
}

bool Matcher::Match(const LiDARObjectListPtr& lidar_obj_list,
           const FusionObjectListPtr& global_obj_list,
           std::map<size_t, int>& lidar_global_map) {
    lidar_global_map.clear();
    for (size_t i = 0; i < lidar_obj_list->objs.size(); ++i) {
        lidar_global_map[i] = -1;
    }
    if (global_obj_list->objs.size() == 0) {
        return true;;
    }

    // compose bipartite weight matrix
    std::vector<std::vector<float>> weights;
    for (size_t i = 0; i < lidar_obj_list->objs.size(); ++i) {
        std::vector<float> row(global_obj_list->objs.size(), 0);
        weights.push_back(row);
        for (size_t j = 0; j < global_obj_list->objs.size(); ++j) {
            weights[i][j] = IoULiDARToFusion(lidar_obj_list->objs[i], global_obj_list->objs[j]);
        }
    }

    // for (size_t i = 0; i < weights.size(); ++i) {
    //     for (size_t j = 0; j < weights[i].size(); ++j) {
    //         std::cout << " " << weights[i][j];
    //     }
    //     std::cout << std::endl;
    // }

    // associate local to global
    auto f = [&](size_t r, size_t c) {return weights[r][c];};
    auto res = munkres_algorithm<float>(lidar_obj_list->objs.size(),
            global_obj_list->objs.size(),
            f);
    auto is_matching = [&](const size_t r, const size_t c) {
        const auto ii = std::find_if(begin(res), end(res), [&](const auto& x) {
                    return (x.first == r and x.second == c);
                });
        return ii != end(res);
    };
    for (size_t i = 0; i < lidar_obj_list->objs.size(); ++i) {
        for (size_t j = 0; j < global_obj_list->objs.size(); ++j) {

            if (is_matching(i, j)) {
                lidar_global_map[i] = j;
                // std::cout << "(i, j)=(" << i << ", " << lidar_global_map[i] << ")" << std::endl;;
            }
        }
    }

    return true;
}

float Matcher::IoUCamToFusion(
        const CameraObjectPtr& cam_obj, const FusionObjectPtr& fusion_obj) {
    float dist = 0.0f;
    if (main_sensor_ == SensorType::LIDAR) {
        BBox2D pred;
        pred.x = cam_obj->ux;
        pred.y = cam_obj->vy;
        pred.width = cam_obj->width;
        pred.height = cam_obj->height;

        BBox3D tgt;
        tgt.x = fusion_obj->x;
        tgt.y = fusion_obj->y;
        tgt.z = fusion_obj->z;
        tgt.width = fusion_obj->width;
        tgt.height = fusion_obj->height;
        tgt.length = fusion_obj->length;

        dist = IoUIn2D(pred, tgt, extrinsic_camera_to_baselink_.inverse(), cam_intrinsic_);
    } else if (main_sensor_ == SensorType::CAMERA) {
        BBox2D pred;
        pred.x = cam_obj->ux;
        pred.y = cam_obj->vy;
        pred.width = cam_obj->width;
        pred.height = cam_obj->height;

        BBox2D tgt;
        tgt.x = fusion_obj->ux;
        tgt.y = fusion_obj->vy;
        tgt.width = fusion_obj->width_2d;
        tgt.height = fusion_obj->height_2d;

        dist = IoUIn2D(pred, tgt); 
    }
    return dist;
}

float Matcher::IoULiDARToFusion(const LiDARObjectPtr& lidar_obj, const FusionObjectPtr& fusion_obj) {
    return std::sqrt(std::pow(lidar_obj->x - fusion_obj->x, 2) +
            std::pow(lidar_obj->y - fusion_obj->y, 2));
    // return IoUIn3D(lidar_obj, fusion_obj);
}

float Matcher::IoURadarToFusion(const RadarObjectPtr& radar_obj, const FusionObjectPtr& fusion_obj) {
    float dist = 0.0f;
    if (main_sensor_ == SensorType::LIDAR) {
        dist = std::sqrt(std::pow(radar_obj->x - fusion_obj->x, 2) +
                std::pow(radar_obj->y - fusion_obj->y, 2));
    } else if (main_sensor_ == SensorType::CAMERA) {
        BBox3D pred;
        pred.x = radar_obj->x;
        pred.y = radar_obj->y;
        pred.z = radar_obj->z;
        pred.width = 1.0;
        pred.height = 1.0;
        pred.length = 1.0;

        BBox2D tgt;
        tgt.x = fusion_obj->ux;
        tgt.y = fusion_obj->vy;
        tgt.width = fusion_obj->width_2d;
        tgt.height = fusion_obj->height_2d;

        dist = IoUIn2D(tgt, pred, extrinsic_radar_to_camera_, cam_intrinsic_);
    }
    return dist;
}

Eigen::Vector3d Matcher::TransformToCameraFrame(
    const Eigen::Vector3d& point, const Eigen::Affine3d& extrinsic) {
    Eigen::Vector3d transformed_point =
        extrinsic.linear() * point + extrinsic.translation();
    Eigen::Vector3d point_in_camera;
    point_in_camera[0] = -transformed_point[1];
    point_in_camera[1] = -transformed_point[2];
    point_in_camera[2] = transformed_point[0];
    return point_in_camera;
}

Eigen::Vector2d Matcher::TransformToImageFrame(const Eigen::Vector3d& point,
                                               const Vector6d& intrinsic) {
    Eigen::Vector2d pt_in_img;
    pt_in_img(0) =
        (intrinsic(0) * point.x() + intrinsic(2)) / point.z() + intrinsic(4);
    pt_in_img(1) =
        (intrinsic(1) * point.y() + intrinsic(3)) / point.z() + intrinsic(5);
    return pt_in_img;
}

Eigen::Vector2d Matcher::SizeOfRadarObjectInImage(
    const RadarObjectPtr& radar_obj,
    const Eigen::Affine3d& extrinsic,
    const Vector6d& intrinsic) {
    std::vector<Eigen::Vector2d> vertices;
    std::vector<std::vector<int>> indices = {
        {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (size_t i = 0; i < indices.size(); ++i) {
        Point3D pt;
        pt.x = radar_obj->x;
        pt.y = radar_obj->y + indices[i][0] * (default_radar_obj_width_ * 0.5);
        pt.z = radar_obj->z + indices[i][1] * (default_radar_obj_height_ * 0.5);
        // std::cout << "point: (" << pt.x << ", " << pt.y << ", " << pt.z <<
        // ")" << std::endl;
        Eigen::Vector2d verty = TransformToImageFrame(
            TransformToCameraFrame(Eigen::Vector3d(pt.x, pt.y, pt.z),
                                   extrinsic),
            intrinsic);
        // std::cout << "point in image: (" << verty(0) << ", " << verty(1) <<
        // ")" << std::endl;
        vertices.push_back(verty);
    }

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();
    for (size_t i = 0; i < vertices.size(); ++i) {
        min_x = std::min(vertices[i](0), min_x);
        max_x = std::max(vertices[i](0), max_x);
        min_y = std::min(vertices[i](1), min_y);
        max_y = std::max(vertices[i](1), max_y);
    }
    min_x = std::max(min_x, 0.0);
    min_y = std::max(min_y, 0.0);
    max_x = std::max(max_x, 0.0);
    max_y = std::max(max_y, 0.0);
    min_x = std::min(min_x, 1920.0);
    min_y = std::min(min_y, 1020.0);
    max_x = std::min(max_x, 1920.0);
    max_y = std::min(max_y, 1020.0);
    return Eigen::Vector2d(max_x - min_x, max_y - min_y);
}


void Matcher::Match(const RadarObjectListPtr& radar_obj_list,
                    const CameraObjectListPtr& cam_obj_list,
                    const FusionObjectListPtr& res_obj_list) {
    if (res_obj_list == NULL) {
        return;
    }

    // Transform targets to Cartesian coordinates and project to camera frame
    std::unordered_map<size_t, BBox2D> radar_boxes_in_img;
    for (size_t i = 0; i < radar_obj_list->objs.size(); ++i) {
        const auto& radar_obj = radar_obj_list->objs[i];
        // Preprocess, filter object not in roi distance
        float dist =
            std::sqrt(std::pow(radar_obj->x, 2) + std::pow(radar_obj->y, 2));
        if (dist <= radar_nearest_dist_thres_ ||
            dist > radar_furthest_dist_thres_) {
            continue;
        }

        // Project radar object to image
        Eigen::Vector3d radar_box_center_point_in_camera =
            TransformToCameraFrame(
                Eigen::Vector3d(radar_obj->x, radar_obj->y, radar_obj->z),
                extrinsic_radar_to_camera_);
        Eigen::Vector2d radar_box_center_point_in_img = TransformToImageFrame(
            radar_box_center_point_in_camera, cam_intrinsic_);
        Eigen::Vector2d radar_box_size = SizeOfRadarObjectInImage(
            radar_obj, extrinsic_radar_to_camera_, cam_intrinsic_);

        // Project the 3d point to 2d pixel coordinates
        // cv::Point2d uv =
        // cam_model_.project3dToPixel(cv::Point3d(target_point_in_camera.x(),
        // target_point_in_camera.y(), target_point_in_camera.z()));
        // radar_objs_in_uv.push_back(uv);
        BBox2D radar_box;
        radar_box.x = radar_box_center_point_in_img(0);
        radar_box.y = radar_box_center_point_in_img(1);
        radar_box.width = radar_box_size(0);
        radar_box.height = radar_box_size(1);
        radar_boxes_in_img.emplace(i, radar_box);
        std::cout << "radar object id: "
            << static_cast<int>(radar_obj->id) << ", base_link coordinate: ("
            << radar_obj->x << ", " << radar_obj->y << ", " << radar_obj->z
            << "), camera coordinate: (" << radar_box_center_point_in_camera(0)
            << ", " << radar_box_center_point_in_camera(1) << ", "
            << radar_box_center_point_in_camera(2) << "), image coordinate: ("
            << radar_box.x << ", " << radar_box.y << ", " << radar_box.width
            << ", " << radar_box.height << ")"
            << ", valid radar object number: " << radar_boxes_in_img.size() << std::endl;
    }

    // Check projected targets against each detected object bounding box.
    // In the case that multiple 3d points alias to be within the same 2d
    // bounding box (bbox), only use the nearest one. If radar and camera
    // data overlap, render the detected object and display radar data as
    // bounding box annotations.
    FusionObjectListPtr fusion_obj_list =
        std::make_shared<FusionObjectList>(FusionObjectList());
    fusion_obj_list->time_ns = cam_obj_list->time_ns;
    fusion_obj_list->frame_id = cam_obj_list->frame_id;
    fusion_obj_list->objs.clear();
    double min_range;
    int ind_min_range;
    for (const auto& object : cam_obj_list->objs) {
        // Set the min range to infinity before checking targets
        min_range = std::numeric_limits<double>::infinity();
        ind_min_range = -1;

        for (const auto& item : radar_boxes_in_img) {
            const auto& idx = item.first;
            const auto& radar_box = item.second;
            BBox2D cam_box;
            cam_box.x = object->ux;
            cam_box.y = object->vy;
            cam_box.width = object->width;
            cam_box.height = object->height;
            // Check if target is within bbox
            // if((radar_obj_in_uv.x <= (object->ux + object->width / 2) +
            // matching_offset_in_ux_) &&
            //         (radar_obj_in_uv.x >= (object->ux - object->width / 2) -
            //         matching_offset_in_ux_) && (radar_obj_in_uv.y <=
            //         (object->vy + object->height / 2) +
            //         matching_offset_in_vy_) && (radar_obj_in_uv.y >=
            //         (object->vy - object->height / 2) -
            //         matching_offset_in_vy_)) {
            if (IoUIn2D(radar_box, cam_box) >= matching_thres_) {
                // Keep track of minimum range and target index
                const auto& radar_obj = radar_obj_list->objs[idx];
                double dist = std::sqrt(std::pow(radar_obj->x, 2) +
                                        std::pow(radar_obj->y, 2));
                if (dist < min_range) {
                    min_range = dist;
                    ind_min_range = idx;
                }
            }
        }

        // If we have a valid radar-camera association, render it
        if (std::isfinite(min_range) && ind_min_range >= 0) {
            const auto& matched_radar_obj = radar_obj_list->objs[ind_min_range];
            std::cout << "matched radar object index= "
                            << ind_min_range << ", id= "
                            << static_cast<int>(matched_radar_obj->id)
                            << ", range= (" << matched_radar_obj->x << ", "
                            << matched_radar_obj->y << ")" << std::endl;

            FusionObjectPtr fusion_obj =
                std::make_shared<FusionObject>(FusionObject());
            fusion_obj->id = matched_radar_obj->id;
            fusion_obj->x = matched_radar_obj->x;
            fusion_obj->y = matched_radar_obj->y;
            fusion_obj->velo_x = matched_radar_obj->velo_x;
            fusion_obj->velo_y = matched_radar_obj->velo_y;
            fusion_obj->ux = object->ux;
            fusion_obj->vy = object->vy;
            fusion_obj->width_2d = object->width;
            fusion_obj->height_2d = object->height;
            fusion_obj->label = object->label;
            fusion_obj->confidence = object->confidence;
            fusion_obj->transformBy(extrinsic_radar_to_baselink_);
            res_obj_list->objs.push_back(fusion_obj);
        }
    }
}

void Matcher::Preprocess(const RadarObjectListPtr& radar_obj_list,
                         const FusionObjectListPtr& fusion_obj_list) {
    fusion_obj_list->time_ns = radar_obj_list->time_ns;
    fusion_obj_list->frame_id = radar_obj_list->frame_id;
    for (size_t i = 0; i < radar_obj_list->objs.size(); ++i) {
        const auto& radar_obj = radar_obj_list->objs[i];
        FusionObjectPtr fusion_obj =
            std::make_shared<FusionObject>(FusionObject());
        fusion_obj->time_ns = radar_obj->time_ns;
        fusion_obj->id = radar_obj->id;
        fusion_obj->x = radar_obj->x;
        fusion_obj->y = radar_obj->y;
        fusion_obj->velo_x = radar_obj->velo_x;
        fusion_obj->velo_y = radar_obj->velo_y;
        fusion_obj->rcs = radar_obj->rcs;
        fusion_obj->length = default_radar_obj_length_;
        fusion_obj->width = default_radar_obj_width_;
        fusion_obj->height = default_radar_obj_height_;
        fusion_obj_list->objs.push_back(fusion_obj);
    }
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
