
#include <glog/logging.h>

#include <TGK/Simulation/VisualWheelCircleSim.h>
#include <TGK/Camera/PinholeRanTanCamera.h>
#include <FilterFusion/ParamLoader.h>
#include <FilterFusion/FilterFusionSystem.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        LOG(ERROR) << "[main]: Please input param_file.";
        return EXIT_FAILURE;
    }

    FLAGS_minloglevel = 3;
    const std::string param_file = argv[1];

    FilterFusion::Parameter params;
    FilterFusion::LoadParam(param_file, &params);

    // Create a camera.
    const std::shared_ptr<TGK::Camera::Camera> camera = std::make_shared<TGK::Camera::PinholeRadTanCamera>(
        params.cam_intrinsic.width, 
        params.cam_intrinsic.height,
        params.cam_intrinsic.fx, params.cam_intrinsic.fy,
        params.cam_intrinsic.cx, params.cam_intrinsic.cy,
        params.cam_intrinsic.k1, params.cam_intrinsic.k2,
        params.cam_intrinsic.p1, params.cam_intrinsic.p2,
        params.cam_intrinsic.k3);

    // Create the simulator.
    TGK::Simulation::VisualWheelCircleSim sim(camera, 
        params.wheel_param.kl, params.wheel_param.kr, params.wheel_param.b - 0.4, // Add noise.
        params.extrinsic.O_R_C, params.extrinsic.O_p_C);

    // Create FilterFusion system.
    FilterFusion::FilterFusionSystem FilterFusion_sys(param_file);

    int cnt = 0;
    constexpr int kSkipCnt = 10;
    Eigen::Matrix3d init_gt_G_R_O;
    Eigen::Vector3d init_gt_G_p_O;
    bool init_flag = false;

    double timestamp;
    Eigen::Matrix3d G_R_O;
    Eigen::Vector3d G_p_O;
    double left_wheel, right_wheel; 
    std::vector<Eigen::Vector2d> features;
    std::vector<long int> feature_ids;
    cv::Mat image;
    
    while (sim.SimOneFrame(&timestamp, &G_R_O, &G_p_O, &left_wheel, &right_wheel, &features, &feature_ids, &image)) {
        FilterFusion_sys.FeedWheelData(timestamp, left_wheel, right_wheel);
        
        if (init_flag == false) {
            init_gt_G_R_O = G_R_O;
            init_gt_G_p_O = G_p_O;
            init_flag = true;
        }
        G_R_O = init_gt_G_R_O.transpose() * G_R_O.eval();
        G_p_O = init_gt_G_R_O.transpose() * (G_p_O.eval() - init_gt_G_p_O);
        FilterFusion_sys.FeedGroundTruth(timestamp, G_R_O, G_p_O);

        if (++cnt <= kSkipCnt) { continue; }
        cnt = 0;
        
        // Add noise.
        for (Eigen::Vector2d& ft : features) {
            ft += Eigen::Vector2d::Random();
        }
        
        FilterFusion_sys.FeedSimData(timestamp, image, features, feature_ids);

        usleep(1e4);
    }
    
    std::cin.ignore();

    return EXIT_SUCCESS;
}