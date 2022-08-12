
#include <glog/logging.h>


#include <TGK/Simulation/VisualWheelCircleSim.h>
#include <TGK/Camera/PinholeRanTanCamera.h>
#include <FilterFusion/ParamLoader.h>
#include <FilterFusion/Visualizer.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        LOG(ERROR) << "[main]: Please input param_file.";
        return EXIT_FAILURE;
    }

    FilterFusion::Parameter params;
    FilterFusion::LoadParam(argv[1], &params);

    // VIZ.
    FilterFusion::Visualizer::Config viz_config;
    std::shared_ptr<FilterFusion::Visualizer> viz = std::make_shared<FilterFusion::Visualizer>(viz_config);

    // Create a camera.
    const std::shared_ptr<TGK::Camera::Camera> camera = std::make_shared<TGK::Camera::PinholeRadTanCamera>(
        params.cam_intrinsic.width, 
        params.cam_intrinsic.height,
        params.cam_intrinsic.fx, params.cam_intrinsic.fy,
        params.cam_intrinsic.cx, params.cam_intrinsic.cy,
        params.cam_intrinsic.k1, params.cam_intrinsic.k2,
        params.cam_intrinsic.p1, params.cam_intrinsic.p2,
        params.cam_intrinsic.k3);

    TGK::Simulation::VisualWheelCircleSim sim(camera, 
        params.wheel_param.kl, params.wheel_param.kr, params.wheel_param.b,
        params.extrinsic.O_R_C, params.extrinsic.O_p_C);

    std::vector<Eigen::Vector3d> map_points;
    std::vector<long int> map_point_ids;
    sim.GetAllMapPoint(&map_points, &map_point_ids);
    viz->DrawFeatures(map_points);

    double timestamp;
    Eigen::Matrix3d G_R_O;
    Eigen::Vector3d G_p_O;
    double left_wheel, right_wheel; 
    std::vector<Eigen::Vector2d> features;
    std::vector<long int> feature_ids;
    cv::Mat image;
    while (sim.SimOneFrame(&timestamp, &G_R_O, &G_p_O, &left_wheel, &right_wheel, &features, &feature_ids, &image)) {
        viz->DrawWheelPose(G_R_O, G_p_O);
        viz->DrawColorImage(image);

        const Eigen::Matrix3d G_R_C = G_R_O * params.extrinsic.O_R_C;
        const Eigen::Vector3d G_p_C = G_p_O + G_R_O * params.extrinsic.O_p_C;
        viz->DrawCameras({std::pair<Eigen::Matrix3d, Eigen::Vector3d>(G_R_C, G_p_C)});
    }
    
    return EXIT_SUCCESS;
}