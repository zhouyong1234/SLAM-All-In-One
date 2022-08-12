//
// Created by jin on 2021/5/19.
//

#include "features_register.h"
#include "data_defination.hpp"

FeaturesRegister::FeaturesRegister(int max_ite, float distance_threshold){
    max_ite_ = max_ite;
    nn_distance_threshold_ = distance_threshold;
}

void FeaturesRegister::setInitial(const Eigen::Affine3d& initial_pose){
    Eigen::Quaterniond q(initial_pose.rotation());
    delta_q_[0] = q.x();
    delta_q_[1] = q.y();
    delta_q_[2] = q.z();
    delta_q_[3] = q.w();
    Eigen::Vector3d trans(initial_pose.translation());
    delta_t_[0] = trans[0];
    delta_t_[1] = trans[1];
    delta_t_[2] = trans[2];
}

void FeaturesRegister::setInitial(double const *delta) {
    for(int index = 0; index < 4; ++index){
        delta_q_[index] = delta[index];
    }
    for(int index = 0; index < 3; ++index){
        delta_t_[index] = delta[index + 4];
    }
}

Scan2ScanMatcher::Scan2ScanMatcher(int max_ite, float distance_threshold, bool deskew, int near_line) :
    FeaturesRegister(max_ite, distance_threshold) {
    deskew_ = deskew;
    NEAR_LINE_NUM = near_line;
}

void Scan2ScanMatcher::pointProjToStart(const PointXYZI& local_point, PointXYZI& result_point){
    Eigen::Quaterniond inter_q(q_);
    Eigen::Vector3d inter_t(t_);
    if(deskew_){
        double inter_coeff = (local_point.intensity - int(local_point.intensity)) / 0.1;// TODO:10Hz雷达
        if(inter_coeff > 1) inter_coeff = 1.0;
        LOG_EVERY_N(INFO, 500) << "cof: " << inter_coeff;
        inter_q = Eigen::Quaterniond::Identity().slerp(inter_coeff, inter_q);
        inter_t = inter_t * inter_coeff;
    }
    result_point = pcl::transformPoint(local_point, Eigen::Affine3f(Eigen::Translation3f(inter_t.cast<float>()) * inter_q.cast<float>()));
}

bool Scan2ScanMatcher::align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map,
                             PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud, Eigen::Affine3d init_pose) {
    if(corner_map->points.size() < 10 || plane_map->points.size() < 50){
        LOG(WARNING) << "Target features are too few!";
        return false;
    }
    target_corner_tree_.setInputCloud(corner_map);
    target_plane_tree_.setInputCloud(plane_map);
    //    LOG(INFO) << "Corner tree size: " << target_corner_tree_.getInputCloud()->size();
    //    LOG(INFO) << "Plane tree size: " << target_plane_tree_.getInputCloud()->size();
    setInitial(init_pose);
    LOG(INFO) << "Start scan2scan align";
    Timer ite_timer("scan2scan solver");
    Eigen::Affine3d last_delta = Eigen::Affine3d::Identity();
    // 最近邻->特征值->判断->添加约束
    for(int ite = 0; ite < max_ite_; ++ite){
        ceres::Problem::Options option;// TODO:参数设置
        ceres::Problem problem(option);
        ceres::LocalParameterization* q_local = new ceres::EigenQuaternionParameterization();
        problem.AddParameterBlock(delta_q_, 4, q_local);
        problem.AddParameterBlock(delta_t_, 3);
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);// 和residual block一起添加

//        Timer t("ite_" + std::to_string(ite));
        std::vector<int> searchIndex;
        std::vector<float> searchSquaDis;
        int corner_constrains = 0, plane_constrains = 0;
        std::vector<ceres::ResidualBlockId> residual_ids;
        ceres::ResidualBlockId block_id;
        for(size_t c_id = 0; c_id < corner_cloud->points.size(); ++c_id){
            PointXYZI point_global;
            const auto& current_local_corner = corner_cloud->points[c_id];
            pointProjToStart(current_local_corner, point_global);
            target_corner_tree_.nearestKSearch(point_global, 1, searchIndex, searchSquaDis);
            if(searchSquaDis.front() > nn_distance_threshold_) continue;// 0.5
            auto neareast_point = corner_map->points[searchIndex.front()];
            int neareast_line = int(neareast_point.intensity);
            int subopt_index = -1;
            double subopt_dis = std::numeric_limits<double>::max();
            for(int index = searchIndex.front() - 1; index >= 0; --index){
                int current_line = int(corner_map->points[index].intensity);
                if(current_line == neareast_line) continue;
                if(current_line < neareast_line - NEAR_LINE_NUM) break;
                double current_dis = pointDis(corner_map->points[index], point_global);
                if(current_dis > subopt_dis) continue;
                subopt_index = index;
                subopt_dis = current_dis;
            }
            for(int index = searchIndex.front() + 1; index < corner_map->points.size(); ++index){
                int current_line = int(corner_map->points[index].intensity);
                if(current_line == neareast_line) continue;
                if(current_line > neareast_line + NEAR_LINE_NUM) break;
                double current_dis = pointDis(corner_map->points[index], point_global);
                if(current_dis > subopt_dis) continue;
                subopt_index = index;
                subopt_dis = current_dis;
            }
            if(subopt_index >= 0 && subopt_dis < nn_distance_threshold_){// 2
                double s = 1;
                if(deskew_){
                    s = std::min((current_local_corner.intensity - int(current_local_corner.intensity)) / 0.1, 1.0);
                }
                ceres::CostFunction* line_cost_func = LineFactor::create(p2v(neareast_point), p2v(corner_map->points[subopt_index]), p2v(corner_cloud->points[c_id]), s);
                block_id = problem.AddResidualBlock(line_cost_func, loss_function, delta_q_, delta_t_);
                residual_ids.emplace_back(block_id);
                corner_constrains++;
            }
        }

        for(size_t p_id = 0; p_id < plane_cloud->points.size(); ++p_id){
            PointXYZI point_global;
            const auto& current_local_plane = plane_cloud->points[p_id];
            pointProjToStart(current_local_plane, point_global);
            target_plane_tree_.nearestKSearch(point_global, 1, searchIndex, searchSquaDis);
            if(searchSquaDis.front() > nn_distance_threshold_) continue;
            auto nearest_point = plane_map->points[searchIndex.front()];
            int neareast_line = int(nearest_point.intensity);
            int online_opt = -1;
            int offline_opt = -1;
            double online_dis = std::numeric_limits<double>::max();
            double offline_dis = std::numeric_limits<double>::max();
            for(int index = searchIndex.front() - 1; index >= 0; --index){ // 这里一定不能包括本身，否则平面中的两个点重合
                int current_line = int(plane_map->points[index].intensity);
                if(current_line == neareast_line){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < online_dis){
                        online_opt = index;
                        online_dis = current_dis;
                    }
                }else if(current_line >= neareast_line - NEAR_LINE_NUM){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < offline_dis){
                        offline_opt = index;
                        offline_dis = current_dis;
                    }
                }else{
                    break;
                }
            }
            for(int index = searchIndex.front() + 1; index < plane_map->points.size(); ++index){
                int current_line = int(plane_map->points[index].intensity);
                if(current_line == neareast_line){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < online_dis){
                        online_opt = index;
                        online_dis = current_dis;
                    }
                }else if(current_line <= neareast_line + NEAR_LINE_NUM){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < offline_dis){
                        offline_opt = index;
                        offline_dis = current_dis;
                    }
                }else{
                    break;
                }
            }
            if(online_opt >= 0 && offline_opt >= 0 && online_dis < nn_distance_threshold_&& offline_dis < nn_distance_threshold_){//
                Eigen::Vector3d second_point = p2v(plane_map->points[online_opt]);
                Eigen::Vector3d third_point = p2v(plane_map->points[offline_opt]);
                Eigen::Vector3d near_point = p2v(nearest_point);
                Eigen::Vector3d norm = (second_point - near_point).cross(third_point - near_point).normalized();
                double d = -(near_point.dot(norm));// norm.dot(x)+d = 0
                double s = 1.0;
                if(deskew_){
                    s = std::min((current_local_plane.intensity - int(current_local_plane.intensity)) / 0.1, 1.0);
                }
                ceres::CostFunction* cost_function = PlaneFactor::create(norm, d, p2v(plane_cloud->points[p_id]), s);
                block_id = problem.AddResidualBlock(cost_function, loss_function, delta_q_, delta_t_);
                residual_ids.emplace_back(block_id);
                plane_constrains++;
            }
        }
        ceres::Solver::Options solve_option;
        solve_option.max_num_iterations = 8;
        solve_option.linear_solver_type = ceres::DENSE_QR;// DENSE_QR DENSE_SCHUR SPARSE_NORMAL_CHOLESKY
        solve_option.check_gradients = false;
        solve_option.gradient_check_relative_precision = 1e-4;
        solve_option.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(solve_option, &problem, &summary);
        LOG(INFO) << "ite " << ite << " constrains: " << corner_constrains << ", " << plane_constrains << ", cost： " << summary.initial_cost << " -> " << summary.final_cost;
        {
            // 评价误差大小
            ceres::Problem::EvaluateOptions eval_ops;
            eval_ops.residual_blocks = residual_ids;
            double total_cost;
            std::vector<double> residuals;
            // residuals的项数等于所有residual维数之和
            problem.Evaluate(eval_ops, &total_cost, &residuals, nullptr, nullptr);
            std::vector<float> residual_norms;
            for(int index = 0; index < residual_ids.size(); ++index){
                if(index < corner_constrains){
                    Eigen::Vector3f v(residuals[3*index], residuals[3*index+1], residuals[3*index+2]);
                    residual_norms.emplace_back(v.norm());
                }else{
                    residual_norms.emplace_back(std::fabs(residuals[3*corner_constrains+(index - corner_constrains)]));// 一定要注意residual可能是负的
                }
            }
            if(residual_ids.size() != residual_norms.size()){
                LOG(ERROR) << "residual_ids.size() != residual_norms.size()";
            }
            double threshold = findKthLargestValue(residual_norms, int(residual_ids.size() * 0.1));
            if(threshold > 0.2){
                LOG(INFO) << "thre: " << threshold;
                std::vector<ceres::ResidualBlockId> tmp_residual_ids;
                int rem = 0;
                for(int index = 0; index < residual_ids.size(); ++index){
                    if(residual_norms.at(index) >= threshold){
                        problem.RemoveResidualBlock(residual_ids.at(index));
                        rem++;
                    }else{
                        tmp_residual_ids.push_back(residual_ids.at(index));
                    }
                }
                LOG(INFO) << "Residual size: " << residual_ids.size() << ", after filter: " << tmp_residual_ids.size();
                ceres::Solve(solve_option, &problem, &summary);
                LOG(INFO) << "Cost 2： " << summary.initial_cost << " -> " << summary.final_cost;
            }
        }
//        t.end();

        Eigen::Affine3d current_delta = Eigen::Translation3d(t_) * q_;
        if(ite > 0){
            Eigen::Affine3d delta_delta_trans = last_delta.inverse() * current_delta;
            if(delta_delta_trans.translation().norm() < 0.005 && Eigen::AngleAxisd(delta_delta_trans.rotation()).angle() < 0.1 * ANG2RAD){
                LOG(INFO) << "Terminate in advance when ite = " << ite;
                break;
            }
        }
        last_delta = current_delta;

//        static int count = 2;
//        PointCloudXYZIPtr tmp_map(new PointCloudXYZI);
//        PointCloudXYZIPtr tmp(new PointCloudXYZI);
//        *tmp = *corner_map;
//        for(auto& p : tmp->points){
//            p.intensity = 255;
//        }
//        *tmp_map += *tmp;
//        *tmp = *plane_map;
//        for(auto& p : tmp->points){
//            p.intensity = 150;
//        }
//        *tmp_map += *tmp;
//        Eigen::Affine3d trans(q_ * Eigen::Translation3d(t_));
//        pcl::transformPointCloud(*corner_cloud, *tmp, trans);
//        for(auto& p : tmp->points){
//            p.intensity = 100;
//        }
//        *tmp_map += *tmp;
//        pcl::transformPointCloud(*plane_cloud, *tmp, trans);
//        for(auto& p : tmp->points){
//            p.intensity = 50;
//        }
//        *tmp_map += *tmp;
//        pcl::io::savePCDFile("/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/" + std::to_string(count) + ".pcd", *tmp_map);
//        count++;
    }
    {
        Eigen::Affine3d current_delta = Eigen::Translation3d(t_) * q_;
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(current_delta, x, y, z, roll, pitch,yaw);
        LOG(INFO) << "Delta: " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw;
    }
    ite_timer.end();
    return true;
}

Scan2MapMatcher::Scan2MapMatcher(int max_ite, float distance_threshold) : FeaturesRegister(max_ite, distance_threshold){

}

void Scan2MapMatcher::pointProject(const PointXYZI &local_point, PointXYZI &result_point) {
    Eigen::Quaterniond inter_q(q_);
    Eigen::Vector3d inter_t(t_);
    result_point = pcl::transformPoint(local_point, Eigen::Affine3f(Eigen::Translation3f(inter_t.cast<float>()) * inter_q.cast<float>()));
}

bool Scan2MapMatcher::align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map, PointCloudXYZIPtr corner_cloud,
                            PointCloudXYZIPtr plane_cloud, Eigen::Affine3d initial_pose) {
    if(corner_map->points.size() < 50 || plane_map->points.size() < 250){
        LOG(WARNING) << "Map features are too few!";
        return false;
    }
    target_corner_tree_.setInputCloud(corner_map);
    target_plane_tree_.setInputCloud(plane_map);
    setInitial(initial_pose);
    LOG(INFO) << "Start scan2map align";
    Timer ite_timer("scan2map solver");
    Eigen::Affine3d last_pose = Eigen::Affine3d::Identity();
    // 最近邻->特征值->判断->添加约束
    for(int ite = 0; ite < max_ite_; ++ite){
        ceres::Problem::Options option;// TODO:参数设置
        ceres::Problem problem(option);
        ceres::LocalParameterization* q_local = new ceres::EigenQuaternionParameterization();
        problem.AddParameterBlock(delta_q_, 4, q_local);
        problem.AddParameterBlock(delta_t_, 3);
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);// 和residual block一起添加

//        Timer t("ite_" + std::to_string(ite));
        std::vector<int> searchIndex;
        std::vector<float> searchSquaDis;
        int corner_constrains = 0, plane_constrains = 0;
        std::vector<ceres::ResidualBlockId> residual_ids;
        ceres::ResidualBlockId block_id;
        for(size_t c_id = 0; c_id < corner_cloud->points.size(); ++c_id){
            PointXYZI point_global;
            const auto& current_local_corner = corner_cloud->points[c_id];
            pointProject(current_local_corner, point_global);
            target_corner_tree_.nearestKSearch(point_global, near_corners_num_, searchIndex, searchSquaDis);
            if(searchSquaDis.back() > nn_distance_threshold_) continue;
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            Eigen::Matrix<double, 3, 3> covariance = Eigen::Matrix<double, 3, 3>::Zero();
            for(int index = 0; index < near_corners_num_; ++index){
                Eigen::Vector3d tmp_v = p2v(corner_map->points[searchIndex[index]]);
                mean += tmp_v;
                covariance += tmp_v * tmp_v.transpose();
            }
            mean /= double(near_corners_num_);
            covariance -= near_corners_num_ * mean * mean.transpose();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
            auto eigen_values = eigen_solver.eigenvalues();
            if(eigen_values[2] > 3 * eigen_values[1]){ // 判断是否为线特征
                Eigen::Vector3d main_direction = eigen_solver.eigenvectors().col(2);
                main_direction.normalize();
                Eigen::Vector3d first_point = mean + main_direction * 0.5;
                Eigen::Vector3d second_point = mean - main_direction * 0.5;
                block_id = problem.AddResidualBlock(LineFactor::create(first_point, second_point, p2v(corner_cloud->points[c_id])), loss_function, delta_q_, delta_t_);
                residual_ids.emplace_back(block_id);
                corner_constrains++;
            }
        }

        for(size_t p_id = 0; p_id < plane_cloud->points.size(); ++p_id){
            PointXYZI point_global;
            const auto& current_local_plane = plane_cloud->points[p_id];
            pointProject(current_local_plane, point_global);
            target_plane_tree_.nearestKSearch(point_global, near_planes_num_, searchIndex, searchSquaDis);
            if(searchSquaDis.back() > nn_distance_threshold_) continue;
            // 拟合平面，一方面是为了构建误差函数，另一方面也是为了判断是否为平面
            Eigen::MatrixXd A(near_planes_num_, 3);
            for(int row_id = 0; row_id < near_planes_num_; ++row_id){
                A.row(row_id) = p2v(plane_map->points[searchIndex[row_id]]);
            }
            Eigen::Vector3d plane_param = A.colPivHouseholderQr().solve(Eigen::VectorXd::Constant(near_planes_num_, -1));
            double d = 1 / plane_param.norm();
            plane_param.normalize();// 修改本身，另外函数normalized()则会返回单位化后的向量
            bool plane_valid = true;
            for(int index = 0; index < near_planes_num_; ++index){
                Eigen::Vector3d neighboring_point = p2v(plane_map->points[searchIndex[index]]);
                if(fabs(plane_param.dot(neighboring_point) + d) > 0.3){ // 不符合面特征
                    plane_valid = false;
                    break;
                }
            }
            if(plane_valid){
                block_id = problem.AddResidualBlock(PlaneFactor::create(plane_param, d, p2v(plane_cloud->points[p_id])), loss_function, delta_q_, delta_t_);// TODO:注意这里输入的一定是local的点
                residual_ids.emplace_back(block_id);
                plane_constrains++;
            }
        }
        ceres::Solver::Options solve_option;
        solve_option.max_num_iterations = 8;
        solve_option.linear_solver_type = ceres::DENSE_QR;// DENSE_QR DENSE_SCHUR SPARSE_NORMAL_CHOLESKY
        solve_option.check_gradients = false;
        solve_option.gradient_check_relative_precision = 1e-4;
        solve_option.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(solve_option, &problem, &summary);
        LOG(INFO) << "ite " << ite << " constrains: " << corner_constrains << ", " << plane_constrains << ", cost： " << summary.initial_cost << " -> " << summary.final_cost;
        {
            // 评价误差大小
            ceres::Problem::EvaluateOptions eval_ops;
            eval_ops.residual_blocks = residual_ids;
            double total_cost;
            std::vector<double> residuals;
            // residuals的项数等于所有residual维数之和
            problem.Evaluate(eval_ops, &total_cost, &residuals, nullptr, nullptr);
            std::vector<float> residual_norms;
            for(int index = 0; index < residual_ids.size(); ++index){
                if(index < corner_constrains){
                    Eigen::Vector3f v(residuals[3*index], residuals[3*index+1], residuals[3*index+2]);
                    residual_norms.emplace_back(v.norm());
                }else{
                    residual_norms.emplace_back(std::fabs(residuals[3*corner_constrains+(index - corner_constrains)]));// 一定要注意residual可能是负的
                }
            }
            if(residual_ids.size() != residual_norms.size()){
                LOG(ERROR) << "residual_ids.size() != residual_norms.size()";
            }
            double threshold = findKthLargestValue(residual_norms, int(residual_ids.size() * 0.1));
            if(threshold > 0.2){
                LOG(INFO) << "thre: " << threshold;
                std::vector<ceres::ResidualBlockId> tmp_residual_ids;
                int rem = 0;
                for(int index = 0; index < residual_ids.size(); ++index){
                    if(residual_norms.at(index) >= threshold){
                        problem.RemoveResidualBlock(residual_ids.at(index));
                        rem++;
                    }else{
                        tmp_residual_ids.push_back(residual_ids.at(index));
                    }
                }
                LOG(INFO) << "Residual size: " << residual_ids.size() << ", after filter: " << tmp_residual_ids.size();
                ceres::Solve(solve_option, &problem, &summary);
                LOG(INFO) << "Cost 2： " << summary.initial_cost << " -> " << summary.final_cost;
            }
        }
//        t.end();

        Eigen::Affine3d current_pose = Eigen::Translation3d(t_) * q_;
        if(ite > 0){
            Eigen::Affine3d delta_pose = last_pose.inverse() * current_pose;
            if(delta_pose.translation().norm() < 0.005 && Eigen::AngleAxisd(delta_pose.rotation()).angle() < 0.1 * ANG2RAD){
                LOG(INFO) << "Terminate in advance when ite = " << ite;
                break;
            }
        }
        last_pose = current_pose;
    }
//    {
//        Eigen::Affine3d current_pose_opt = Eigen::Translation3d(t_) * q_;
//        double x, y, z, roll, pitch, yaw;
//        pcl::getTranslationAndEulerAngles(current_pose_opt, x, y, z, roll, pitch,yaw);
//        LOG(INFO) << "current opt pose: " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw;
//    }
    ite_timer.end();
    return true;
}




