//
// Created by jin on 2021/6/1.
//

#include "lab_slam.h"

LabSLAM::LabSLAM() {
    ros::NodeHandle nh("~");
    cloud_msg_sub_ = nh.subscribe(topic_name_, 5, &LabSLAM::msgCallback, this);
    loop_closure_pub_ = nh.advertise<visualization_msgs::Marker>("detected_loop_closures", 1);

    gtsam::ISAM2Params params;
    params.relinearizeSkip = 1;
    params.relinearizeThreshold = 0.001;
    optimizer_ = std::make_shared<gtsam::ISAM2>(params);

    global_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_map", 1);
}

void LabSLAM::msgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    LOG(INFO) << "Current msg timestamp: " << std::fixed << std::setprecision(6) << msg->header.stamp.toSec();
    velodyne_msg_mutex_.lock();
    velodyne_msgs_.emplace_back(msg);
    velodyne_msg_mutex_.unlock();
    msg_condit_var_.notify_all();
    LOG(INFO) << "Current cloud size: " << velodyne_msgs_.size();
}

void LabSLAM::preprocessWork() {
    while(1){
        sensor_msgs::PointCloud2::ConstPtr cloud(new sensor_msgs::PointCloud2);
        std::unique_lock<std::mutex> msg_unique_lock(velodyne_msg_mutex_);
        msg_condit_var_.wait(msg_unique_lock, [this, &cloud]()->bool{if(velodyne_msgs_.empty()) {return false;} cloud = velodyne_msgs_.front(); velodyne_msgs_.pop_front(); return true;});
        msg_unique_lock.unlock();
        DataGroupPtr data_group(new DataGroup);
        Timer t("preprocess work");
        pre_processor_.work(cloud, data_group);
        t.end();
        data_mutex_.lock();
        data_.push_back(data_group);
//        LOG(INFO) << "Data group size after preprocess: " << data_.size();
        data_mutex_.unlock();
        data_condit_var_.notify_all();
    }
}

void LabSLAM::lidarOdoWork() {
    while(1){
        DataGroupPtr data_group(new DataGroup);
        std::unique_lock<std::mutex> data_unique_lock(data_mutex_);
        data_condit_var_.wait(data_unique_lock, [this, &data_group](){if(data_.empty()) return false; data_group = data_.front(); data_.pop_front(); return true;});
        data_unique_lock.unlock();

        KeyFramePtr key_frame = nullptr;
        bool is_key_frame = lidar_odo_.work(data_group, key_frames_, key_frame);
        if(is_key_frame){
            key_frames_.emplace_back(std::move(key_frame));
        }
        if(is_key_frame)
            loopClosure();
    }
}

void LabSLAM::publishGlobalMap(){
    while(true){
        std::unique_lock<std::mutex> lock(publish_global_map_mutex_);
        publish_global_map_var_.wait(lock);
        int opt_size = new_node_index_;
        Timer t("Publish global map");
        PointCloudXYZIPtr global_map(new PointCloudXYZI);
        PointCloudXYZIPtr trans_cloud(new PointCloudXYZI);
        PointCloudXYZIPtr tmp_cloud(new PointCloudXYZI);
        pcl::VoxelGrid<PointXYZI> voxel_filter;
        voxel_filter.setLeafSize(0.1, 0.1, 0.1);
        for(int index = 0; index < opt_size; ++index){
            trans_cloud->clear();
            tmp_cloud->clear();
            pcl::transformPointCloud(*key_frames_.at(index)->corner_cloud_, *tmp_cloud, key_frames_.at(index)->pose_);
            *trans_cloud += *tmp_cloud;
            tmp_cloud->clear();
            pcl::transformPointCloud(*key_frames_.at(index)->plane_cloud_, *tmp_cloud, key_frames_.at(index)->pose_);
            *trans_cloud += *tmp_cloud;
            voxel_filter.setInputCloud(trans_cloud);
            voxel_filter.filter(*trans_cloud);
            *global_map += *trans_cloud;
        }
        voxel_filter.setInputCloud(global_map);
        voxel_filter.filter(*global_map);
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map, map_msg);
        map_msg.header.frame_id = "map";
        global_map_pub_.publish(map_msg);
        t.end();
        publish_global_map_flag_ = false;
    }
}

void LabSLAM::loopClosure(){
        LoopClosurePair closure_pair;
        Timer t("loop detection");
        if(loop_detection_.detect(key_frames_, closure_pair)){
            LOG(INFO) << "Success loop closure: " << closure_pair.target_index << " vs " << closure_pair.source_index;
            {
                static visualization_msgs::Marker m;
                m.type = visualization_msgs::Marker::LINE_LIST;
                m.header.frame_id = "map";
                m.header.seq = 1;
                m.action = visualization_msgs::Marker::ADD;
                m.color.g = m.color.a = 1.0;
                m.pose.orientation.w = 1.0;
                m.scale.x = 0.1;
                geometry_msgs::Point p;
                Eigen::Vector3d target_position = key_frames_.at(closure_pair.target_index)->pose_.translation().matrix();
                p.x = target_position.x();
                p.y = target_position.y();
                p.z = target_position.z();
                m.points.push_back(p);
                Eigen::Vector3d source_position = key_frames_.at(closure_pair.source_index)->pose_.translation().matrix();
                p.x = source_position.x();
                p.y = source_position.y();
                p.z = source_position.z();
                m.points.push_back(p);
                loop_closure_pub_.publish(m);
            }

            // 在图中增加新的节点，里程计约束和回环约束，进行优化
            if(new_node_index_ == 0){
                Pose3 init_pose(key_frames_.front()->pose_.matrix());
                values_.insert(X(0), init_pose);
                noiseModel::Diagonal::shared_ptr prior_pose_noise = noiseModel::Diagonal::Sigmas((Vector6() << 1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4).finished());
                graph_.add(PriorFactor<Pose3>(X(0), init_pose, prior_pose_noise));
                new_node_index_ = 1;
            }
            noiseModel::Diagonal::shared_ptr odometry_between_noise = noiseModel::Diagonal::Sigmas((Vector6() << 1e-2, 1e-2, 1e-2, 3e-2, 3e-2, 3e-2).finished());
            while(new_node_index_ < key_frames_.size()){
                Pose3 new_pose(key_frames_.at(new_node_index_)->pose_.matrix());
                values_.insert(X(new_node_index_), new_pose);
                Pose3 last_pose(key_frames_.at(new_node_index_ - 1)->pose_.matrix());
                graph_.add(BetweenFactor<Pose3>(X(new_node_index_ - 1), X(new_node_index_), last_pose.between(new_pose), odometry_between_noise));
                new_node_index_++;
            }
            noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Sigmas((Vector6() << 1e-3, 1e-3, 1e-3, 5e-3, 5e-3, 5e-3).finished());
            graph_.add(BetweenFactor<Pose3>(X(closure_pair.target_index), X(closure_pair.source_index), Pose3(closure_pair.trans.matrix()), loop_noise));
            optimizer_->update(graph_, values_);
            optimizer_->update();
            optimizer_->update();
            optimizer_->update();
            optimizer_->update();
//            optimizer_->print();
            Values result = optimizer_->calculateBestEstimate();
//            result.print();
            for(int index = 0; index < new_node_index_; ++index){
                Pose3 pose = result.at<Pose3>(X(index));
//                pose.print();
                if(key_frames_.at(index)->index_ != index){
                    LOG(FATAL) << "Wrong index: " << index << " vs " << key_frames_.at(index)->index_;
                }
                LOG(INFO) << index << ": " << key_frames_.at(index)->pose_.matrix();
                key_frames_.at(index)->pose_ = pose.matrix();
                LOG(INFO) << index << ": " << key_frames_.at(index)->pose_.matrix();
            }
            // 修改odometry drift
            Eigen::Affine3d last_odo_pose = key_frames_.back()->odo_pose_;
            Eigen::Affine3d last_opt_pose = key_frames_.back()->pose_;
            lidar_odo_.setCurrentPoseOpt(last_opt_pose);
            lidar_odo_.setOdoDrift(last_opt_pose * last_odo_pose.inverse());
            // 清空graph
            graph_.resize(0);
            values_.clear();
            LOG(WARNING) << "gtsam optimization finished...";
            // 发布全局最优地图
//            publish_global_map_mutex_.lock();
//            publish_global_map_flag_ = true;
//            publish_global_map_mutex_.unlock();
            publish_global_map_var_.notify_all();
        }else{
            LOG(INFO) << "No loop";
        }
//        key_frame_mutex_.unlock();
//    }
        t.end();
}

int main(int argc, char** argv){
    google::InitGoogleLogging("LabSLAM");
    google::SetLogDestination(google::INFO, "/home/touchair/output/log");
    google::SetStderrLogging(google::INFO);
    ros::init(argc, argv, "LabSLAM");
    LabSLAM lab_slam;
    std::thread pre_process(&LabSLAM::preprocessWork, &lab_slam);
    std::thread lidar_odo(&LabSLAM::lidarOdoWork, &lab_slam);
//    std::thread loop_closure(&LabSLAM::loopClosure, &lab_slam);
    std::thread loop_closure(&LabSLAM::publishGlobalMap, &lab_slam);
    ros::spin();
    return 0;
}