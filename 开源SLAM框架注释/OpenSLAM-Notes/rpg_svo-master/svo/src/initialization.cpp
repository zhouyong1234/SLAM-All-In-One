// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

namespace svo {
namespace initialization {

// 添加第一帧
InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  // 重置
  reset();
  // 提取特征点
  detectFeatures(frame_ref, px_ref_, f_ref_);
  if(px_ref_.size() < 100)
  {// 确保数量
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }
  // 保存参考帧 特征点
  frame_ref_ = frame_ref;
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  return SUCCESS;
}

// 添加第二帧
InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
  // 光流跟踪
  trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
  // 光流跟踪点数要求
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");
  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;
  // 获取视差中值
  double disparity = vk::getMedian(disparities_);
  // 确保视差
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;
  // RANSAC 估计单应矩阵
  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_);
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");
  // 确保内点数目
  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }
  // 缩放地图，固定尺度
  // Rescale the map such that the mean scene depth is equal to the specified scale
  // 获取深度信息
  vector<double> depth_vec;
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  // 深度中值  
  double scene_depth_median = vk::getMedian(depth_vec);
  // 缩放系数
  double scale = Config::mapScale()/scene_depth_median;
  // 当前帧的位置
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  // 尺度缩放
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

  // For each inlier create 3D point and add feature in both frames
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {// 遍历所有内点
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    // 视野内
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      // 缩放后的地图点位置
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      // new point 
      Point* new_point = new Point(pos);
      // new feature 
      Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
      // 当前帧添加特征
      frame_cur->addFeature(ftr_cur);
      // 地图点关联特征
      new_point->addFrameRef(ftr_cur);
      // 参考帧对应的特征
      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      // 添加到参考帧
      frame_ref_->addFeature(ftr_ref);
      // 添加到地图点
      new_point->addFrameRef(ftr_ref);
    }
  }
  return SUCCESS;
}

void KltHomographyInit::reset()
{
  // 跟踪点 参考帧 清除
  px_cur_.clear();
  frame_ref_.reset();
}

// 提取特征
void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  // 保存特征数据
  Features new_features;
  // 特征提取
  feature_detection::FastDetector detector(
      frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
  detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

  // 对于所有特征，保存信息到 px_vec f_vec 中
  // now for all maximum corners, initialize a new seed
  px_vec.clear(); px_vec.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    f_vec.push_back(ftr->f);
    delete ftr;
  });
}

// 光流跟踪
void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities)
{
  // 一些参数
  const double klt_win_size = 30.0;
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  vector<uchar> status;
  vector<float> error;
  vector<float> min_eig_vec;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
  // opencv的光流函数
  cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
                           px_ref, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);
  // 保存跟踪特征点数据
  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();
  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
    // 
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
    // 视差计算
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}

// 单应矩阵计算
void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref)
{
  // 跟踪结果赋值
  vector<Vector2d > uv_ref(f_ref.size());
  vector<Vector2d > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    uv_ref[i] = vk::project2d(f_ref[i]);
    uv_cur[i] = vk::project2d(f_cur[i]);
  }
  // 单应计算
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  // 相对位姿关系                   
  T_cur_from_ref = Homography.T_c2_from_c1;
}


} // namespace initialization
} // namespace svo
