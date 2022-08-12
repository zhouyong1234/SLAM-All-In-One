//
// Created by jin on 2021/5/16.
//

#include <queue>
#include "pre_processor.h"
#include "cost_function.hpp"
#include "data_defination.hpp"
//#define GROUND_SEGMENTATION

PreProcessor::PreProcessor() {
    ros::NodeHandle nh("~");
//    cloud_sub_ = nh.subscribe(topic_name_, 5, &PreProcessor::cloudCallback, this);
    full_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("preprocess/full_cloud", 10);
    ground_and_clusters_pub_ = nh.advertise<sensor_msgs::PointCloud2>("preprocess/ground_and_clusters_cloud", 10);
    corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("preprocess/corner_points", 2);
    planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("preprocess/plane_points", 2);
    less_corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("preprocess/less_corner_points", 2);
    less_planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("preprocess/less_plane_points", 2);
    cloud_.reset(new PointCloudVelodyne);
    cloud_image_.reset(new PointCloudXYZI);
    full_cloud_.reset(new PointCloudXYZI);
    ground_and_clusters_.reset(new PointCloudXYZI);
    corner_points_.reset(new PointCloudXYZI);
    less_corner_points_.reset(new PointCloudXYZI);
    plane_points_.reset(new PointCloudXYZI);
    less_plane_points_.reset(new PointCloudXYZI);
    scan_down_sample_filter_.setLeafSize(0.8, 0.8, 0.8);
    std::pair<int, int> mov;
    mov.first = -1;
    mov.second = 0;
    direction_.push_back(mov);
    mov.first = 1;
    direction_.push_back(mov);
    mov.first = 0;
    mov.second = -1;
    direction_.push_back(mov);
    mov.second = 1;
    direction_.push_back(mov);
    ROS_INFO("Preprocessor has been created...");
}

void PreProcessor::resetParam() {
//    if(cloud_) cloud_->clear();
//    cloud_ = velodyne_cloud;
    cloud_->clear();
    cloud_image_->clear();
    cloud_image_->resize(N_SCAN_ * HORIZON_SCAN_);// 有的位置在投影后可能是未定义的状态，与range_image_对应// TODO：加速
    range_image_.resize(N_SCAN_, HORIZON_SCAN_);
//    range_image_.setOnes();
//    range_image_ *= -1;
    range_image_.setConstant(-1);// -1代表无效
    ground_label_.resize(N_SCAN_, HORIZON_SCAN_);
    ground_label_.setZero();// 初始化为0,无效点为-1,有效为1
    cluster_label_.resize(N_SCAN_, HORIZON_SCAN_);
    cluster_label_.setZero();// 初始化为0,地面点和无效点设置为-1不被聚类，>=1为有效编号
    component_index_ = 1;
    full_cloud_->clear();
    ground_and_clusters_->clear();
    start_index_.clear();
    start_index_.resize(N_SCAN_);
    end_index_.clear();
    end_index_.resize(N_SCAN_);
    col_index_.clear();
    ranges_.clear();
    ground_flags_.clear();
    curve_id_.clear();
    selected_.clear();
    corner_points_->clear();
    less_corner_points_->clear();
    plane_points_->clear();
    less_plane_points_->clear();
}

template<typename Point>
void PreProcessor::nanFilter(const typename pcl::PointCloud<Point> &cloud_in, typename pcl::PointCloud<Point> &cloud_out) {
    int origin_count = cloud_in.size();
    int count = 0;
    if (&cloud_in != &cloud_out) {
        cloud_out.resize(cloud_in.size());
    }
    for (size_t index = 0; index < cloud_in.size(); ++index) {
        const auto &point = cloud_in[index];
        if (point.x == NAN || point.y == NAN || point.z == NAN) continue;
        cloud_out[count++] = point;
    }
    cloud_out.resize(count);
    if(origin_count - count != 0){
        LOG(INFO) << "Filtered " << origin_count - count << " nan points.";
    }
}

void PreProcessor::rangeFilter(const PointCloudVelodynePtr &cloud_in, const PointCloudVelodynePtr &cloud_out,
                               float minimum_range, float maximum_range) {

    int origin_count = cloud_in->size();
    if (cloud_out != cloud_in) {
        cloud_out->header = cloud_in->header;
        cloud_out->points.resize(cloud_in->points.size());
    }
    int index = 0;
    float square_min = minimum_range * minimum_range;
    float square_max = maximum_range * maximum_range;
    for (size_t id = 0; id < cloud_in->points.size(); ++id) {
        const auto &point = cloud_in->points[id];
        float square_range = point.x * point.x + point.y * point.y + point.z * point.z;
        if (square_range < square_min || square_range > square_max) {
            continue;
        }
        cloud_out->points[index++] = cloud_in->points[id];
    }
    cloud_out->resize(index);// 自动调整points/height/width
    if(index != origin_count){
        LOG(INFO) << "Filtered " << origin_count - index << " points by range filter...";
    }

}


void PreProcessor::filter() {
//    std::vector<int> index;
//    pcl::removeNaNFromPointCloud<PointVelodyne>(*cloud_, *cloud_, index);
    nanFilter(*cloud_, *cloud_);
    rangeFilter(cloud_, cloud_, 0.5, 150);
//    std::vector<int> count(16, 0);
//    std::vector<float> angles;
//    for(const auto& point : cloud_->points){
//        count[point.ring]++;
//        if(point.ring == 5){
//            angles.push_back(atan2(point.y, point.x) * RAD2ANG);
//        }
//    }
//    for(const auto& c : count){
//        std::cout << c << ", ";
//    }
//    std::cout << std::endl;
//    std::sort(angles.begin(), angles.end());
//    float min_angle = 1000;
//    for(int index = 1; index < angles.size(); ++index){
////        if(angles[index] - angles[index - 1] < min_angle){
////            min_angle = angles[index] - angles[index - 1];
////        }
//        if(index % 100 == 0){
//            LOG(INFO) << angles[index] - angles[index - 1];
//        }
//    }
//    std::cout << min_angle << std::endl;
}


void PreProcessor::pointIndex(const PointVelodyne &point, int &i, int &j) {
    i = static_cast<int>(point.ring);
//    float resolution = 0.0;
//    j = HORIZON_SCAN_ * 0.5 - round((atan2(point.x, point.y) - M_PI_2) / (2 * M_PI) * HORIZON_SCAN_);
    j = int((atan2(point.x, point.y) + M_PI) / (2 * M_PI) * HORIZON_SCAN_);
    if (j < 0) {
        LOG(WARNING) << "J < 0: " << j;
        j += HORIZON_SCAN_;
    } else if (j >= HORIZON_SCAN_) {
        LOG(WARNING) << "J > HORIZON_SCAN_: " << j;
        j -= HORIZON_SCAN_;
    }
}

void PreProcessor::projectToImage() {
    size_t cloud_size = cloud_->points.size();
//    LOG(INFO) << "original points size: " << cloud_size;
    int count = 0;
    float max_time = 0;
    for (size_t index = 0; index < cloud_size; ++index) {
        int i, j;
        const auto &point = cloud_->points[index];
        pointIndex(point, i, j);
//        LOG_EVERY_N(INFO, 100) << point.time;
        if (range_image_(i, j) == -1) {
            range_image_(i, j) = pointRange(point);
            auto &im_point = cloud_image_->points[i * HORIZON_SCAN_ + j];
            im_point.x = point.x;
            im_point.y = point.y;
            im_point.z = point.z;
            im_point.intensity = point.ring + point.time;// point.time最大在0.1左右
            if (point.time > max_time) {
                max_time = point.time;
            }
        } else {
//            LOG(INFO) << "Exists!";
            count++;
        }
    }

//     {
//         // 输出深度图，从上至下行号递增；从右侧开始顺时针旋转，列数递增
//         static int index = 0;
//         cv::Mat m(cv::Size(HORIZON_SCAN_, N_SCAN_), CV_8UC1);
//         float max_depth = 0.01;
//         for (int row = 0; row < N_SCAN_; ++row) {
//             for (int col = 0; col < HORIZON_SCAN_; ++col) {
//                 max_depth = range_image_(row, col) > max_depth ? range_image_(row, col) : max_depth;
//             }
//         }
// //        LOG(INFO) << "MAX DEPTH: " << max_depth;
//         for (int row = 0; row < N_SCAN_; ++row) {
//             for (int col = 0; col < HORIZON_SCAN_; ++col) {
//                 if (range_image_(row, col) <= 0) {
//                     m.at<uchar>(N_SCAN_ - 1 - row, col) = 0;
//                 } else {
//                     float ratio = range_image_(row, col) / max_depth;
//                     if (ratio > 1) {
//                         LOG(WARNING) << "Ratio > 1!";
//                         ratio = 1.0;
//                     }
//                     m.at<uchar>(N_SCAN_ - 1 - row, col) = int(ratio * 255);
//                 }
//             }
//         }
//         cv::applyColorMap(m, m, cv::COLORMAP_RAINBOW);
//         std::string path =
//                 "/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/depth_pic/" + std::to_string(index) + ".bmp";
//         cv::imwrite(path, m);
//         index++;
//     }
//    LOG(INFO) << "---------------MAX TIMESTAMP: " << max_time;
//    LOG(INFO) << "Duplicate count: " << count;
}

void PreProcessor::markGround() {
    for (int col = 0; col < HORIZON_SCAN_; ++col) {
        for (int row = 0; row < ground_scan_; ++row) {
            float current_depth = range_image_(row, col);
            float next_depth = range_image_(row, col + 1);
            if (current_depth == -1 || next_depth == -1) {
                ground_label_(row, col) = -1;// 存在无效像素
                continue;
            }
            PointXYZI current_point = cloud_image_->points[row * HORIZON_SCAN_ + col];
            PointXYZI next_point = cloud_image_->points[(row + 1) * HORIZON_SCAN_ + col];
            float height_diff = fabs(current_point.z - next_point.z);
            float horizon_diff = sqrt(pow(current_point.x - next_point.x, 2) + pow(current_point.y - next_point.y, 2));
            if (atan(height_diff / horizon_diff) < tan(10.0 / 180.0 * M_PI)) {
                ground_label_(row, col) = 1;
            }
        }
    }
    for (int col = 0; col < HORIZON_SCAN_; ++col) {
        for (int row = 0; row < N_SCAN_; ++row) {
            if (range_image_(row, col) == -1 || ground_label_(row, col) == 1) { // 无效像素和地面像素不参加聚类
                cluster_label_(row, col) = -1;
            }
        }
    }
    return;
}

void PreProcessor::componentCluster(int row, int col) {
    std::deque<std::pair<int, int> > q;// 存放等待被考察周边的坐标
    std::vector<std::pair<int, int> > coordinates;// 用来记录聚类成功的所有像素坐标，为的是在某种条件下修改其componentIndex为无效cluster
    std::vector<bool> lines(N_SCAN_, false);
    cluster_label_(row, col) = component_index_;
    coordinates.emplace_back(std::make_pair(row, col));
    lines[row] = true;
    q.emplace_back(std::make_pair(row, col));
    while (!q.empty()) {
        auto current_pos = q.front();
        q.pop_front();
        for (int index = 0; index < 4; ++index) {
            auto mov = direction_[index];
            std::pair<int, int> next_pos(current_pos.first + mov.first, current_pos.second + mov.second);
            if (next_pos.first < 0 || next_pos.first >= N_SCAN_ || next_pos.second < 0 ||
                next_pos.second >= HORIZON_SCAN_)
                continue;
            if (cluster_label_(next_pos.first, next_pos.second) != 0) {
                continue;
            }
            float resolution = (mov.first == 0) ? 0.2 * ANG2RAD : ANG2RAD;// TODO：垂直1度？
            float current_depth = range_image_(current_pos.first, current_pos.second);
            float next_depth = range_image_(next_pos.first, next_pos.second);
            float large_depth = std::max(current_depth, next_depth);
            float small_depth = std::min(current_depth, next_depth);
            float angle = std::atan2(sin(resolution) * small_depth, large_depth - cos(resolution) * small_depth);
            if (angle < 0) {
                LOG(FATAL) << "Minus angle!!!";
            }
            if (angle > 20.0 / 180.0 * M_PI) { // 下调有助于聚类更多的较大入射角面上的点云，上调有助于筛选更稳定的面上的点
                cluster_label_(next_pos.first, next_pos.second) = component_index_;
                coordinates.emplace_back(next_pos);
                lines[next_pos.first] = true;
                q.push_back(next_pos);// 等待考察其边界
            }
        }
    }
    int count = 0;
    for (const auto &flag : lines) {
        if (flag) {
            count++;
        }
    }
    if (coordinates.size() > 30 || (coordinates.size() > 15 && count > 4))
        return;
    for (const auto &cor : coordinates) {
        cluster_label_(cor.first, cor.second) = -1;// 9999
    }
    return;
}

void PreProcessor::markGroundAndComponentCluster() {
    LOG(INFO) << "/////////////////////Start seg";
    markGround();// 标记地面，在cluster_label_中将地面和无效像素标记为-1
    for (int row = 0; row < N_SCAN_; ++row) {
        for (int col = 0; col < HORIZON_SCAN_; ++col) {
            if (cluster_label_(row, col) == 0) { // 初始化状态，且不是地面和无效像素
                componentCluster(row, col);
            }
        }
    }
}

void PreProcessor::rearrangeBackCloud() {
    int count1 = 0;
    int count2 = 0;
    int count3 = 0;
    for (int row = 0; row < N_SCAN_; ++row) {
        start_index_[row] = full_cloud_->size() + 5;
        for (int col = 0; col < HORIZON_SCAN_; ++col) {
            if (range_image_(row, col) != -1) {
                count1++;
            }
            if (ground_label_(row, col) == 1) {
                count2++;
            }
            if (cluster_label_(row, col) > 0) {
                count3++;
            }
            // compact
#ifdef GROUND_SEGMENTATION
            if (range_image_(row, col) != -1 && (ground_label_(row, col) == 1 || cluster_label_(row, col) > 0))
#else
                if(range_image_(row, col) != -1)
#endif
            {
                auto point = cloud_image_->points[row * HORIZON_SCAN_ + col];
                full_cloud_->push_back(point);
                ranges_.push_back(range_image_(row, col));
                col_index_.push_back(col);
#ifdef GROUND_SEGMENTATION
                if (ground_label_(row, col) == 1) {
                    ground_flags_.push_back(true);
//                    point.intensity = 50;
                } else {
                    ground_flags_.push_back(false);
//                    point.intensity = 100;
                }
//                if(ground_label_(row, col) == 1 && cluster_label_(row, col) > 0){
//                    LOG(WARNING) << "Ground and cluster!!!";
//                }
//                ground_and_clusters_->push_back(point);
#endif
            }
        }
        end_index_[row] = full_cloud_->size() - 1 - 5;
//        LOG(INFO) << "row " << row << " points: " << end_index_[row] - start_index_[row];// 高处的线有效点数只有300+
    }
//    LOG(INFO) << count1 << ", " << count2 << ", " << count3;
    LOG(INFO) << "full cloud size: " << full_cloud_->size();
}

// 计算曲率
// 需要注意的是每个线两侧的边界处是没有意义的，后面不会使用
void PreProcessor::calcuCurvature() {
    curve_id_.resize(full_cloud_->size());
    for (size_t index = 5; index < full_cloud_->size() - 5; ++index) {
        float diff = ranges_[index - 5] + ranges_[index - 4] + ranges_[index - 3] + ranges_[index - 2] \
 + ranges_[index - 1] - 10 * ranges_[index] + ranges_[index + 1] + ranges_[index + 2] + ranges_[index + 3] \
 + ranges_[index + 4] + ranges_[index + 5];
        curve_id_[index] = std::make_pair(diff * diff, index);// 注意这里一定是平方形式的正值
    }
}

void PreProcessor::excludeOcculded() {
    selected_.resize(full_cloud_->size(), false);
    for (int row = 0; row < N_SCAN_; ++row) {
        int start = start_index_[row];
        int end = end_index_[row];
        for (int index = start; index < end; index++) {
            if (!selected_[index]) {
                // 遮挡，列号足够接近才会出现遮挡
                if (abs(col_index_[index] - col_index_[index + 1]) < 10) {
                    if (ranges_[index] - ranges_[index + 1] < -0.3) { // 大索引号被遮挡
                        int offset = 0;
                        while (offset < 5 && index + 1 + offset <= end) {
                            selected_[index + 1 + offset] = true;
                            offset++;
                        }
                    } else if (ranges_[index] - ranges_[index + 1] > 0.3) {
                        int offset = 0;
                        while (offset < 5 && index - offset >= start) {
                            selected_[index - offset] = true;
                            offset++;
                        }
                    }
                }
                // 噪声
                float diff1 = std::fabs(ranges_[index] - ranges_[index - 1]);
                float diff2 = std::fabs(ranges_[index] - ranges_[index + 1]);
                float range = ranges_[index];
                if (diff1 > 0.02 * range && diff2 > 0.02 * range) {// 拖尾也会被去掉0.02 * range
                    selected_[index] = true;
                }
                // 防止单个点在列向聚类，且横向相邻点深度近似
                if (abs(col_index_[index] - col_index_[index - 1]) > 5 &&
                    abs(col_index_[index] - col_index_[index + 1]) > 5) {
                    selected_[index] = true;
                }
            }
        }
    }
}

void PreProcessor::extractFeatures() {
    for (int row = 0; row < N_SCAN_; ++row) {
        int start = start_index_[row];
        int end = end_index_[row];// 这里每一行的起止索引，已经避开了两侧的边界
        if (end - start + 1 < 12) continue;
        PointCloudXYZIPtr plane_scan(new PointCloudXYZI);
        plane_scan->clear();
        for (int sector = 0; sector < 6; ++sector) {
            size_t sector_start_id = (start * (6 - sector) + end * sector) / 6;
            size_t sector_end_id = (start * (5 - sector) + end * (sector + 1)) / 6;
            std::sort(curve_id_.begin() + sector_start_id, curve_id_.begin() + sector_end_id, curve_comp());//
            // 提取最大的20个，
            int corner_count = 0;
            for (size_t curve_id = sector_start_id; curve_id < sector_end_id; ++curve_id) {
                size_t point_id = curve_id_[curve_id].second;
                float curve = curve_id_[curve_id].first;
                if (curve < corner_thre_) break;// 后面的只会更小
#ifdef GROUND_SEGMENTATION
                if (!selected_[point_id] && !ground_flags_.at(point_id)) {
#else
                    if(!selected_[point_id]){
#endif
                    if (corner_count < 6) {
                        corner_points_->push_back(full_cloud_->points[point_id]);
                        less_corner_points_->push_back(full_cloud_->points[point_id]);
                    } else if (corner_count < 15) {
                        less_corner_points_->push_back(full_cloud_->points[point_id]);
                    }
                    if (++corner_count >= 15) break;
                    // 附近都不能再被选中
                    selected_[point_id] = true;
                    int offset = 1;
                    while (offset <= 5 && point_id + offset < sector_end_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id + offset]) < 10) {
                        selected_[point_id + offset] = true;
                        offset++;
                    }
                    offset = 1;
                    while (offset <= 5 && point_id - offset >= sector_start_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id - offset]) < 10) {
                        selected_[point_id - offset] = true;
                        offset++;
                    }
                }
            }
            int plane_count = 0;
            for (int curve_id = sector_end_id - 1; curve_id >= sector_start_id; --curve_id) {
                size_t point_id = curve_id_[curve_id].second;
                float curve = curve_id_[curve_id].first;
                if (curve > plane_thre_) break;
#ifdef GROUND_SEGMENTATION
                if (!selected_[point_id] && ground_flags_.at(point_id)) {
#else
                    if(!selected_[point_id]){
#endif
                    plane_points_->push_back(full_cloud_->points[point_id]);
                    less_plane_points_->push_back(full_cloud_->points[point_id]);
                    // 附近都不能再被选中
                    selected_[point_id] = true;
                    int offset = 1;
                    while (offset <= 5 && point_id + offset < sector_end_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id + offset]) < 10) {
                        selected_[point_id + offset] = true;
                        offset++;
                    }
                    offset = 1;
                    while (offset <= 5 && point_id - offset >= sector_start_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id - offset]) < 10) {
                        selected_[point_id - offset] = true;
                        offset++;
                    }
                    if (++plane_count > 6) break;
                }
            }
            for (int curve_id = sector_end_id - 1; curve_id >= sector_start_id; --curve_id) {
                size_t point_id = curve_id_[curve_id].second;
                if (curve_id_[curve_id].first > plane_thre_) break;
                if (!selected_[point_id]) {
                    plane_scan->push_back(full_cloud_->points[point_id]);
                }
            }
        }
        // 对同一个scan上的面点降采样然后保存到plane_points_
        scan_down_sample_filter_.setInputCloud(plane_scan);
        scan_down_sample_filter_.filter(*plane_scan);
        *less_plane_points_ += *plane_scan;
    }
    LOG(INFO) << "Features count: " << corner_points_->size() << ", " << less_corner_points_->size() << ", "
              << plane_points_->size() << ", " << less_plane_points_->size();
}

void PreProcessor::publishSaveFeas(std_msgs::Header h, const DataGroupPtr &data_group) {
    sensor_msgs::PointCloud2 temp_cloud_msg;
    if (corners_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*corner_points_, temp_cloud_msg);
        temp_cloud_msg.header = h;
        corners_pub_.publish(temp_cloud_msg);
    }
    if (planes_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*plane_points_, temp_cloud_msg);
        temp_cloud_msg.header = h;
        planes_pub_.publish(temp_cloud_msg);
    }
    if (less_corners_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*less_corner_points_, temp_cloud_msg);
        temp_cloud_msg.header = h;
        less_corners_pub_.publish(temp_cloud_msg);
    }
    if (less_planes_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*less_plane_points_, temp_cloud_msg);
        temp_cloud_msg.header = h;
        less_planes_pub_.publish(temp_cloud_msg);
    }

    double time_stamp = h.stamp.toSec();
//    LOG(INFO) << "Time stamp: " << std::fixed << std::setprecision(3) << time_stamp << " seconds.";
    data_group->time_stamp = time_stamp;
    data_group->corner_cloud = std::move(
            corner_points_);// corner_points_依然是一个可以指向cloud的指针变量，可以被取地址，但是其内容已经被置空，需要重新被reset才能使用
    corner_points_.reset(new PointCloudXYZI);
    data_group->plane_cloud = std::move(plane_points_);
    plane_points_.reset(new PointCloudXYZI);
    data_group->less_corner_cloud = std::move(less_corner_points_);
    less_corner_points_.reset(new PointCloudXYZI);
    data_group->less_plane_cloud = std::move(less_plane_points_);
    less_plane_points_.reset(new PointCloudXYZI);
    data_group->h = h;
    data_group->is_finished = true;
//    LOG(INFO) << "corner points num: " << data_group->corner_cloud->points.size() << ", plane points num: " << data_group->plane_cloud->points.size();
//    LOG(INFO) << "less corner points num: " << data_group->less_corner_cloud->points.size() << ", less plane points num: " << data_group->less_plane_cloud->points.size();
}

// 发布地面和聚类点
void PreProcessor::publishPointCloudFullCloud(std_msgs::Header h) {
    sensor_msgs::PointCloud2 cloud_msg;
    if (full_cloud_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*full_cloud_, cloud_msg);
        cloud_msg.header = h;
        full_cloud_pub_.publish(cloud_msg);// 发布地面和聚类点，强度整数部分为原始强度
    }
//    if(ground_and_clusters_pub_.getNumSubscribers() != 0){
//        pcl::toROSMsg(*ground_and_clusters_, cloud_msg);
//        cloud_msg.header = h;
//        ground_and_clusters_pub_.publish(cloud_msg);// 为了区分地面和聚类点，强度不同
//    }
}

//void PreProcessor::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
//    msg_mutex_.lock();
//    msgs_.emplace_back(cloud_msg);
//    LOG(INFO) << "Msg size after push: " << msgs_.size();
//    msg_mutex_.unlock();
//}

void PreProcessor::work(const sensor_msgs::PointCloud2::ConstPtr &velodyne_msg, const DataGroupPtr &datagroup) {
//    LOG(INFO) << "Handle new msg---------------";
    // debug
    datagroup->cloud_msg = velodyne_msg;
    resetParam();
    static int has_timestamp = -1;
    if (has_timestamp == -1) {
        for (const auto &field : velodyne_msg->fields) {
            LOG(INFO) << field.name;
            if (field.name == "time") {
                has_timestamp = 1;
                LOG(INFO) << "Have timestamp";
                break;
            }
        }
        if (has_timestamp == -1) {
            has_timestamp = 0;
            LOG(INFO) << "Do not have timestamp";
        }
    }
    if (has_timestamp == 0) {
        PointCloudVelodyne2 tmp_cloud;
        pcl::fromROSMsg(*velodyne_msg, tmp_cloud);
        cloud_->clear();
        std::vector<int> count(16, 0);
        for (const auto &point : tmp_cloud) {
            PointVelodyne p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.intensity = point.intensity;
            p.ring = point.ring;
            p.time = 0.0;
            cloud_->push_back(p);
        }
    } else {
        pcl::fromROSMsg(*velodyne_msg, *cloud_);
    }
//    for(const auto& point : cloud_->points){
//        LOG_EVERY_N(INFO, 100) << point.ring;
//    }
    static int index = 0;
//    pcl::io::savePCDFileBinary("/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/pcd/" + std::to_string(index) + ".pcd", *cloud_);
    index++;
    filter();
    // 投影到image中，会存在Nan位置，同时点的类型从Velodyne->PointXYZI
    projectToImage();
    // 提取地面，对非地面点进行聚类
    // 这里的地面和聚类其实并没有本质上的区别，都是为了获取稳定的面上的点
    // 非地面点可以通过小的入射角进行聚类筛选，但是地面点普遍存在较大的入射角，因此在纵向上进行特殊的处理和判断
#ifdef GROUND_SEGMENTATION
//#pragma message("seg ")
    markGroundAndComponentCluster();
    LOG(INFO) << "/////////////////////////Do ground segmentation!";
#else
    //#pragma message("not seg")
#endif
    // 更加紧凑，同时记录下每个线的起止索引，每个点所在的列
    rearrangeBackCloud();
    publishPointCloudFullCloud(velodyne_msg->header);
    // 计算曲率
    calcuCurvature();
    // 处理遮挡和噪声
    excludeOcculded();
    extractFeatures();
//    LOG(INFO) << "Time stamp: " << std::fixed << std::setprecision(3) << time_stamp << " seconds.";
    publishSaveFeas(velodyne_msg->header, datagroup);
}
