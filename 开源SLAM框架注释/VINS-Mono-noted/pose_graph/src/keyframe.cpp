#include "keyframe.h"

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// create keyframe online
/**
 * @brief Construct a new Key Frame:: Key Frame object，创建一个KF对象，计算已有特征点的描述子，同时额外提取fast角点并计算描述子
 * 
 * @param[in] _time_stamp KF的时间戳
 * @param[in] _index KF的索引
 * @param[in] _vio_T_w_i vio节点中的位姿
 * @param[in] _vio_R_w_i 
 * @param[in] _image 对应的原图
 * @param[in] _point_3d KF对应VIO节点中的世界坐标
 * @param[in] _point_2d_uv 像素坐标
 * @param[in] _point_2d_norm 归一化相机坐标
 * @param[in] _point_id 地图点的idx
 * @param[in] _sequence 序列号
 */
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
		           vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_norm,
		           vector<double> &_point_id, int _sequence)
{
	time_stamp = _time_stamp;
	index = _index;
	vio_T_w_i = _vio_T_w_i;
	vio_R_w_i = _vio_R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
	origin_vio_T = vio_T_w_i;		
	origin_vio_R = vio_R_w_i;
	image = _image.clone();
	cv::resize(image, thumbnail, cv::Size(80, 60)); // 这个缩小尺寸应该是为了可视化
	point_3d = _point_3d;
	point_2d_uv = _point_2d_uv;
	point_2d_norm = _point_2d_norm;
	point_id = _point_id;
	has_loop = false;	// 默认还没有检测到回环
	loop_index = -1;
	has_fast_point = false;
	loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
	sequence = _sequence;
	computeWindowBRIEFPoint();
	computeBRIEFPoint();
	if(!DEBUG_IMAGE)
		image.release();
}

// load previous keyframe
/**
 * @brief Construct a new Key Frame:: Key Frame object从已有的地图中加载KF，就是一些简单的赋值操作
 * 
 * @param[in] _time_stamp 
 * @param[in] _index 
 * @param[in] _vio_T_w_i 
 * @param[in] _vio_R_w_i 
 * @param[in] _T_w_i 
 * @param[in] _R_w_i 
 * @param[in] _image 
 * @param[in] _loop_index 
 * @param[in] _loop_info 
 * @param[in] _keypoints 
 * @param[in] _keypoints_norm 
 * @param[in] _brief_descriptors 
 */
KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
					cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
					vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors)
{
	time_stamp = _time_stamp;
	index = _index;
	//vio_T_w_i = _vio_T_w_i;
	//vio_R_w_i = _vio_R_w_i;
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = _T_w_i;
	R_w_i = _R_w_i;
	if (DEBUG_IMAGE)
	{
		image = _image.clone();
		cv::resize(image, thumbnail, cv::Size(80, 60));
	}
	if (_loop_index != -1)
		has_loop = true;
	else
		has_loop = false;
	loop_index = _loop_index;
	loop_info = _loop_info;
	has_fast_point = false;
	sequence = 0;
	keypoints = _keypoints;
	keypoints_norm = _keypoints_norm;
	brief_descriptors = _brief_descriptors;
}

/**
 * @brief 计算已有特征点的描述子
 * 
 */
void KeyFrame::computeWindowBRIEFPoint()
{
	// 定义一个描述子计算的对象
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	for(int i = 0; i < (int)point_2d_uv.size(); i++)
	{
	    cv::KeyPoint key;
	    key.pt = point_2d_uv[i];	// 像素坐标用来计算描述子
	    window_keypoints.push_back(key);	// 转成opencv格式的特征点坐标
	}
	extractor(image, window_keypoints, window_brief_descriptors);	// 计算VIO节点提取的特征点的描述子
}

// 额外提取fast特征点并计算描述子
void KeyFrame::computeBRIEFPoint()
{
	BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
	const int fast_th = 20; // corner detector response threshold
	if(1)
		cv::FAST(image, keypoints, fast_th, true);	// 提取fast特征点
	else
	{
		vector<cv::Point2f> tmp_pts;
		cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
		for(int i = 0; i < (int)tmp_pts.size(); i++)
		{
		    cv::KeyPoint key;
		    key.pt = tmp_pts[i];
		    keypoints.push_back(key);
		}
	}
	extractor(image, keypoints, brief_descriptors);
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		Eigen::Vector3d tmp_p;
		m_camera->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);	// 将像素坐标得到去畸变的归一化相机坐标
		cv::KeyPoint tmp_norm;
		tmp_norm.pt = cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z());	// 再归一化一下
		keypoints_norm.push_back(tmp_norm);
	}
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);	// 调用dbow的接口计算描述子
}

/**
 * @brief 暴力匹配法，通过遍历所有的候选描述子得到最佳匹配
 * 
 * @param[in] window_descriptor 当前帧的一个描述子
 * @param[in] descriptors_old 回环帧的描述子集合
 * @param[in] keypoints_old 回环帧像素坐标集合
 * @param[in] keypoints_old_norm 回环帧归一化坐标集合
 * @param[out] best_match 最佳匹配的像素坐标
 * @param[out] best_match_norm 最佳匹配的归一化相机坐标
 * @return true 
 * @return false 
 */
bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
		// 遍历回环帧所有描述子
    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);	// 计算两个描述子之间的得分
        if(dis < bestDist)	// 找到匹配得分最高的
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80)
    {
      best_match = keypoints_old[bestIndex].pt;
      best_match_norm = keypoints_old_norm[bestIndex].pt;
      return true;
    }
    else
      return false;
}

/**
 * @brief 将当前帧的描述子依次和回环帧描述子进行匹配，得到匹配结果
 * 
 * @param[out] matched_2d_old 匹配回环帧点的像素坐标集合
 * @param[out] matched_2d_old_norm 匹配回环帧点的归一化相机坐标集合
 * @param[out] status 状态位
 * @param[in] descriptors_old 回环帧的描述子集合
 * @param[in] keypoints_old 回环帧的像素坐标
 * @param[in] keypoints_old_norm 回环帧的归一化坐标
 */
void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
								std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm)
{
    // 遍历当前的光流用的角点来进行描述子匹配
		for(int i = 0; i < (int)window_brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
				// 进行暴力匹配
        if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
          status.push_back(1);	// 匹配上了状态位置1
        else
          status.push_back(0);	// 没匹配上状态位置0
        matched_2d_old.push_back(pt);	// 对应的像素坐标和归一化坐标存起来
        matched_2d_old_norm.push_back(pt_norm);
    }

}


void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                      const std::vector<cv::Point2f> &matched_2d_old_norm,
                                      vector<uchar> &status)
{
	int n = (int)matched_2d_cur_norm.size();
	for (int i = 0; i < n; i++)
		status.push_back(0);
    if (n >= 8)
    {
        vector<cv::Point2f> tmp_cur(n), tmp_old(n);
        for (int i = 0; i < (int)matched_2d_cur_norm.size(); i++)
        {
            double FOCAL_LENGTH = 460.0;
            double tmp_x, tmp_y;
            tmp_x = FOCAL_LENGTH * matched_2d_cur_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_cur_norm[i].y + ROW / 2.0;
            tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

            tmp_x = FOCAL_LENGTH * matched_2d_old_norm[i].x + COL / 2.0;
            tmp_y = FOCAL_LENGTH * matched_2d_old_norm[i].y + ROW / 2.0;
            tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
        }
        cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
    }
}

/**
 * @brief 通过PNP对当前帧和回环是否构成回环进行校验
 * 
 * @param[in] matched_2d_old_norm 回环帧2d归一化坐标
 * @param[in] matched_3d 当前帧3d地图点
 * @param[out] status 
 * @param[out] PnP_T_old 
 * @param[out] PnP_R_old 
 */
void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
	//for (int i = 0; i < matched_3d.size(); i++)
	//	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
	//printf("match size %d \n", matched_3d.size());
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);	// 设置单位阵
    Matrix3d R_inital;
    Vector3d P_inital;
    Matrix3d R_w_c = origin_vio_R * qic;	// 转成相机坐标系
    Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

		// Twc -> Tcw
    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);
		// 转成opencv格式
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    cv::Mat inliers;
    TicToc t_pnp_ransac;

    if (CV_MAJOR_VERSION < 3)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
        else
            // 使用当前帧位姿作为起始位姿，考虑到回环帧距离起始帧并不远
						solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    }
		// 初始化状态位全是0
    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
        status.push_back(0);
		// inlier部分置1
    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    }
		// 转回eigen，以及Tcw -> Twc -> Twi
    cv::Rodrigues(rvec, r);
    Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);
    R_w_c_old = R_pnp.transpose();
    Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(t, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    PnP_R_old = R_w_c_old * qic.transpose();	// 这是是回环帧在VIO坐标系下的位姿
    PnP_T_old = T_w_c_old - PnP_R_old * tic;

}

/**
 * @brief 寻找两帧之间联系，确定是否回环
 * 
 * @param[in] old_kf 
 * @return true 
 * @return false 
 */
bool KeyFrame::findConnection(KeyFrame* old_kf)
{
	TicToc tmp_t;
	//printf("find Connection\n");
	vector<cv::Point2f> matched_2d_cur, matched_2d_old;
	vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
	vector<cv::Point3f> matched_3d;
	vector<double> matched_id;
	vector<uchar> status;

	matched_3d = point_3d;
	matched_2d_cur = point_2d_uv;
	matched_2d_cur_norm = point_2d_norm;
	matched_id = point_id;

	TicToc t_match;
	#if 0
		if (DEBUG_IMAGE)    
	    {
	        cv::Mat gray_img, loop_match_img;
	        cv::Mat old_img = old_kf->image;
	        cv::hconcat(image, old_img, gray_img);
	        cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)point_2d_uv.size(); i++)
	        {
	            cv::Point2f cur_pt = point_2d_uv[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)old_kf->keypoints.size(); i++)
	        {
	            cv::Point2f old_pt = old_kf->keypoints[i].pt;
	            old_pt.x += COL;
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        ostringstream path;
	        path << "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "0raw_point.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
	#endif
	//printf("search by des\n");
	// 通过描述子来check
	searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm);
	// 操作跟光流追踪类似，根据状态位进行筛选
	reduceVector(matched_2d_cur, status);	// 当前帧的像素坐标
	reduceVector(matched_2d_old, status);	// 回环帧的像素坐标
	reduceVector(matched_2d_cur_norm, status);	// 当前帧的归一化坐标
	reduceVector(matched_2d_old_norm, status);	// 回环帧的归一化坐标
	reduceVector(matched_3d, status);	// 当前帧对应的VIO中3d世界坐标
	reduceVector(matched_id, status);	// 当前帧对应的地图点索引
	//printf("search by des finish\n");

	#if 0 
		if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap);
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path, path1, path2;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	        /*
	        path1 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_1.jpg";
	        cv::imwrite( path1.str().c_str(), image);
	        path2 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_2.jpg";
	        cv::imwrite( path2.str().c_str(), old_img);	        
	        */
	        
	    }
	#endif
	status.clear();
	/*
	FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status);
	reduceVector(matched_2d_cur, status);
	reduceVector(matched_2d_old, status);
	reduceVector(matched_2d_cur_norm, status);
	reduceVector(matched_2d_old_norm, status);
	reduceVector(matched_3d, status);
	reduceVector(matched_id, status);
	*/
	#if 0
		if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap) ;
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "2fundamental_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
	#endif
	Eigen::Vector3d PnP_T_old;
	Eigen::Matrix3d PnP_R_old;
	Eigen::Vector3d relative_t;
	Quaterniond relative_q;
	double relative_yaw;
	// 判断匹配上的点数目大小够不够
	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
		status.clear();
			// 进行PNP几何校验，利用当前帧的地图点（3d）和回环帧的归一化相机坐标（2d）进行计算
	    PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
			// 根据状态位进行瘦身
	    reduceVector(matched_2d_cur, status);
	    reduceVector(matched_2d_old, status);
	    reduceVector(matched_2d_cur_norm, status);
	    reduceVector(matched_2d_old_norm, status);
	    reduceVector(matched_3d, status);
	    reduceVector(matched_id, status);
	    #if 1
	    	if (DEBUG_IMAGE)
	        {
	        	int gap = 10;
	        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
	            cv::Mat gray_img, loop_match_img;
	            cv::Mat old_img = old_kf->image;
	            cv::hconcat(image, gap_image, gap_image);
	            cv::hconcat(gap_image, old_img, gray_img);
	            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	            for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	            {
	                cv::Point2f cur_pt = matched_2d_cur[i];
	                cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	            }
	            for(int i = 0; i< (int)matched_2d_old.size(); i++)
	            {
	                cv::Point2f old_pt = matched_2d_old[i];
	                old_pt.x += (COL + gap);
	                cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	            }
	            for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	            {
	                cv::Point2f old_pt = matched_2d_old[i];
	                old_pt.x += (COL + gap) ;
	                cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 2, 8, 0);
	            }
	            cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
	            putText(notation, "current frame: " + to_string(index) + "  sequence: " + to_string(sequence), cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);

	            putText(notation, "previous frame: " + to_string(old_kf->index) + "  sequence: " + to_string(old_kf->sequence), cv::Point2f(20 + COL + gap, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
	            cv::vconcat(notation, loop_match_img, loop_match_img);

	            /*
	            ostringstream path;
	            path <<  "/home/tony-ws1/raw_data/loop_image/"
	                    << index << "-"
	                    << old_kf->index << "-" << "3pnp_match.jpg";
	            cv::imwrite( path.str().c_str(), loop_match_img);
	            */
	            if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	            {
	            	/*
	            	cv::imshow("loop connection",loop_match_img);  
	            	cv::waitKey(10);  
	            	*/
	            	cv::Mat thumbimage;
	            	cv::resize(loop_match_img, thumbimage, cv::Size(loop_match_img.cols / 2, loop_match_img.rows / 2));
	    	    	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", thumbimage).toImageMsg();
	                msg->header.stamp = ros::Time(time_stamp);
	    	    	pub_match_img.publish(msg);
	            }
	        }
	    #endif
	}
	// 根据PNP的内点数进行判断，足够才认为回环可能成功
	if ((int)matched_2d_cur.size() > MIN_LOOP_NUM)
	{
	    //  T_old_w * T_w_cur = T_old_cur; old:回环帧 cur：当前帧
			relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
	    relative_q = PnP_R_old.transpose() * origin_vio_R;
			// 计算yaw角差
	    relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());
	    //printf("PNP relative\n");
	    //cout << "pnp relative_t " << relative_t.transpose() << endl;
	    //cout << "pnp relative_yaw " << relative_yaw << endl;
			// 合理范围之内才认为有效
	    if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0)
	    {

	    	has_loop = true;	// 至此才认为找到了回环
	    	loop_index = old_kf->index;	// 确定回环帧的idx，以及两帧之间相对位姿
	    	loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
	    	             relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
	    	             relative_yaw;
	    	if(FAST_RELOCALIZATION)
	    	{
			    sensor_msgs::PointCloud msg_match_points;
			    msg_match_points.header.stamp = ros::Time(time_stamp);
					// 回环帧2d归一化坐标
			    for (int i = 0; i < (int)matched_2d_old_norm.size(); i++)
			    {
		            geometry_msgs::Point32 p;
		            p.x = matched_2d_old_norm[i].x;
		            p.y = matched_2d_old_norm[i].y;
		            p.z = matched_id[i];	// 对应的VIO地图点的id
		            msg_match_points.points.push_back(p);
			    }
			    Eigen::Vector3d T = old_kf->T_w_i; 	// 回环帧的pose
			    Eigen::Matrix3d R = old_kf->R_w_i;
			    Quaterniond Q(R);
			    sensor_msgs::ChannelFloat32 t_q_index;
			    t_q_index.values.push_back(T.x());
			    t_q_index.values.push_back(T.y());
			    t_q_index.values.push_back(T.z());
			    t_q_index.values.push_back(Q.w());
			    t_q_index.values.push_back(Q.x());
			    t_q_index.values.push_back(Q.y());
			    t_q_index.values.push_back(Q.z());
			    t_q_index.values.push_back(index);	// 当前帧的索引
			    msg_match_points.channels.push_back(t_q_index);
			    pub_match_points.publish(msg_match_points);
	    	}
	        return true;
	    }
	}
	//printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(), t_match.toc());
	return false;
}


int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = vio_T_w_i;
    _R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

/**
 * @brief 更新一些vio位姿，将回环修正的累计误差补偿进来
 * 
 * @param[in] _T_w_i 
 * @param[in] _R_w_i 
 */
void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
	vio_T_w_i = _T_w_i;
	vio_R_w_i = _R_w_i;
	T_w_i = vio_T_w_i;
	R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
    return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopRelativeQ()
{
    return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw()
{
    return loop_info(7);
}

void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
{
	if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
	{
		//printf("update loop info\n");
		loop_info = _loop_info;
	}
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);	// dbow加载pattern用于后续计算描述子
}


