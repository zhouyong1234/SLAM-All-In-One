#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

/**
 * @brief 得到有效的地图点的数目
 * 
 * @return int 
 */
int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

/**
 * @brief 增加特征点信息，同时检查上一帧是否时关键帧
 * 
 * @param[in] frame_count 
 * @param[in] image 
 * @param[in] td 
 * @return true 
 * @return false 
 */

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    // 遍历每个特征点
    for (auto &id_pts : image)
    {
        // 用特征点信息构造一个对象
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        // 在已有的id中寻找是否是有相同的特征点
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });
        // 这是一个新的特征点
        if (it == feature.end())
        {
            // 在特征点管理器中，新创建一个特征点id，这里的frame_count就是该特征点在滑窗中的当前位置，作为这个特征点的起始位置
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        // 如果这是一个已有的特征点，就在对应的“组织”下增加一个帧属性
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;   // 追踪到上一帧的特征点数目
        }
    }
    // 前两帧都设置为KF，追踪过少也认为是KF
    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        // 计算的实际上是frame_count-1,也就是前一帧是否为关键帧
        // 因此起始帧至少得是frame_count - 2,同时至少覆盖到frame_count - 1帧
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }
    // 这个和上一帧没有相同的特征点
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        // 看看平均视差是否超过一个阈值
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

/**
 * @brief 得到同时被frame_count_l frame_count_r帧看到的特征点在各自的坐标
 * 
 * @param[in] frame_count_l 
 * @param[in] frame_count_r 
 * @return vector<pair<Vector3d, Vector3d>> 
 */

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        // 保证需要的特征点被这两帧都观察到
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            // 获得在feature_per_frame中的索引
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));  // 返回相机坐标系下的坐标对
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

/**
 * @brief 移除一些不能被三角化的点
 * 
 */
void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

/**
 * @brief 把给定的深度赋值给各个特征点作为逆深度
 * 
 * @param[in] x 
 */
void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

/**
 * @brief 得到特征点的逆深度
 * 
 * @return VectorXd 
 */
VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

/**
 * @brief 利用观测到该特征点的所有位姿来三角化特征点
 * 
 * @param[in] Ps 
 * @param[in] tic 
 * @param[in] ric 
 */
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    // 遍历每一个特征点
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)  // 代表已经三角化过了
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        // Twi -> Twc,第一个观察到这个特征点的KF的位姿
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();
        // 遍历所有看到这个特征点的KF
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            // 得到该KF的相机坐标系位姿
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            // T_w_cj -> T_c0_cj
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            // T_c0_cj -> T_cj_c0相当于把c0当作世界系
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            // 构建超定方程的其中两个方程
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        // 求解齐次坐标下的深度
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];
        // 得到的深度值实际上就是第一个观察到这个特征点的相机坐标系下的深度值
        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH; // 具体太近就设置成默认值
        }

    }
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

/**
 * @brief 
 * 
 * @param[in] marg_R  被移除的位姿
 * @param[in] marg_P 
 * @param[in] new_R    转接地图点的位姿
 * @param[in] new_P 
 */

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)   // 如果不是被移除的帧看到，那么该地图点对应的起始帧id减一
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;    // 取出归一化相机坐标系坐标
            it->feature_per_frame.erase(it->feature_per_frame.begin()); // 该点不再被原来的第一帧看到，因此从中移除
            if (it->feature_per_frame.size() < 2)   // 如果这个地图点没有至少被两帧看到
            {
                feature.erase(it);  // 那他就没有存在的价值了
                continue;
            }
            else    // 进行管辖权的转交
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth; // 实际相机坐标系下的坐标
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;  // 转到世界坐标系下
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);  // 转到新的最老帧的相机坐标系下
                double dep_j = pts_j(2);
                if (dep_j > 0)  // 看看深度是否有效
                    it->estimated_depth = dep_j;    // 有效的话就得到在现在最老帧下的深度值
                else
                    it->estimated_depth = INIT_DEPTH;   // 无效就设置默认值
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}
/**
 * @brief 这个还没初始化结束，因此相比刚才，不进行地图点新的深度的换算，因为此时还有进行视觉惯性对齐
 * 
 */
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

// 对margin倒数第二帧进行处理
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count) // 如果地图点被最后一帧看到，由于滑窗，他的起始帧减1
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;  // 倒数第二帧在这个地图点对应KF vector的idx
            if (it->endFrame() < frame_count - 1)   // 如果该地图点不能被倒数第二帧看到，那没什么好做的
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j); // 能被倒数第二帧看到，erase掉这个索引
            if (it->feature_per_frame.size() == 0)  // 如果这个地图点没有别的观测了
                feature.erase(it);  // 就没有存在的价值了
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    // 找到相邻两帧
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;  // 归一化相机坐标系的坐标差
    // 当都是归一化坐标系时，他们两个都是一样的
    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}