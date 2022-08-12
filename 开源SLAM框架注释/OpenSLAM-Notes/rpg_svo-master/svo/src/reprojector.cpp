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

#include <algorithm>
#include <stdexcept>
#include <svo/reprojector.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {

Reprojector::Reprojector(vk::AbstractCamera* cam, Map& map) :
    map_(map)
{
  initializeGrid(cam);
}

Reprojector::~Reprojector()
{
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
}

void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
  grid_.cell_size = Config::gridSize();
  grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);
  grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);
  grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
  grid_.cell_order.resize(grid_.cells.size());
  for(size_t i=0; i<grid_.cells.size(); ++i)
    grid_.cell_order[i] = i;
  random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration!
}

// 重置栅格，删除上一次匹配的数据
void Reprojector::resetGrid()
{
  // 重置数据
  n_matches_ = 0;
  n_trials_ = 0;
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

// 地图投影
void Reprojector::reprojectMap(
    FramePtr frame,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
  // 重置栅格
  resetGrid();

  // Identify those Keyframes which share a common field of view.
  // 计算离当前帧有共视关系帧
  SVO_START_TIMER("reproject_kfs");
  list< pair<FramePtr,double> > close_kfs;
  map_.getCloseKeyframes(frame, close_kfs);
  
  // 排序
  // Sort KFs with overlap according to their closeness
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
                 boost::bind(&std::pair<FramePtr, double>::second, _2));

  // Reproject all mappoints of the closest N kfs with overlap. We only store
  // in which grid cell the points fall.
  // 遍历所有的关键帧
  size_t n = 0;
  overlap_kfs.reserve(options_.max_n_kfs);
  for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
      it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
  {
    // 记录共视关键帧
    FramePtr ref_frame = it_frame->first;
    overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

    // Try to reproject each mappoint that the other KF observes
    for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
        it_ftr!=ite_ftr; ++it_ftr)
    {// 遍历共视关键帧的所有特征点
      // check if the feature has a mappoint assigned
      if((*it_ftr)->point == NULL)
        continue;

      // make sure we project a point only once
      if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
        continue;
      (*it_ftr)->point->last_projected_kf_id_ = frame->id_;
      // 将特征点投影到当前帧上
      if(reprojectPoint(frame, (*it_ftr)->point))
        overlap_kfs.back().second++;
    }
  }
  SVO_STOP_TIMER("reproject_kfs");

  // Now project all point candidates
  SVO_START_TIMER("reproject_candidates");
  {
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
    auto it=map_.point_candidates_.candidates_.begin();
    while(it!=map_.point_candidates_.candidates_.end())
    {// 遍历地图中所有的候选点
      if(!reprojectPoint(frame, it->first))
      {// 如果投影不成功
        it->first->n_failed_reproj_ += 3;
        // 投影不成功次数太多就删除该点
        if(it->first->n_failed_reproj_ > 30)
        {
          map_.point_candidates_.deleteCandidate(*it);
          it = map_.point_candidates_.candidates_.erase(it);
          continue;
        }
      }
      ++it;
    }
  } // unlock the mutex when out of scope
  SVO_STOP_TIMER("reproject_candidates");
  
  // 从之前投影的点中，根据栅格划分，一个栅格选一个最优的点进行匹配
  // Now we go through each grid cell and select one point to match.
  // At the end, we should have at maximum one reprojected point per cell.
  SVO_START_TIMER("feature_align");
  for(size_t i=0; i<grid_.cells.size(); ++i)
  {// 遍历所有的栅格挑选点
    // we prefer good quality points over unkown quality (more likely to match)
    // and unknown quality over candidates (position not optimized)
    if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
      ++n_matches_;
    if(n_matches_ > (size_t) Config::maxFts())
      break;
  }
  SVO_STOP_TIMER("feature_align");
}

// 根据点的类型排序
bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

// 从栅格中挑一个特征点出来
bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
  // 对栅格内的特征点类型排序，优先匹配成熟的地图点
  cell.sort(boost::bind(&Reprojector::pointQualityComparator, _1, _2));
  Cell::iterator it=cell.begin();
  while(it!=cell.end())
  {// 遍历所有点
    ++n_trials_;
    // 删除点去掉
    if(it->pt->type_ == Point::TYPE_DELETED)
    {
      it = cell.erase(it);
      continue;
    }
    // 采用类似光流方法，得到投影特征点位置
    bool found_match = true;
    if(options_.find_match_direct)
      found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);
    if(!found_match)
    {// 匹配结果不够好
      // 次数多了就删除
      it->pt->n_failed_reproj_++;
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map_.safeDeletePoint(it->pt);
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        map_.point_candidates_.deleteCandidatePoint(it->pt);
      it = cell.erase(it);
      continue;
    }
    it->pt->n_succeeded_reproj_++;
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;

    Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
    frame->addFeature(new_feature);

    // Here we add a reference in the feature to the 3D point, the other way
    // round is only done if this frame is selected as keyframe.
    new_feature->point = it->pt;

    if(matcher_.ref_ftr_->type == Feature::EDGELET)
    {
      new_feature->type = Feature::EDGELET;
      new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
      new_feature->grad.normalize();
    }

    // If the keyframe is selected and we reproject the rest, we don't have to
    // check this point anymore.
    it = cell.erase(it);
    // 找到了一个
    // Maximum one point per cell.
    return true;
  }
  return false;
}

// 特征点投影，把关键帧的点 + 候选点进行投影
bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
  // 计算世界坐标系
  Vector2d px(frame->w2c(point->pos_));
  // 如果在当前相机视野范围内
  if(frame->cam_->isInFrame(px.cast<int>(), 8)) // 8px is the patch size in the matcher
  {
    // 计算属于的栅格
    const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols
                + static_cast<int>(px[0]/grid_.cell_size);
    // 构造候选点，放入栅格中            
    grid_.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  return false;
}

} // namespace svo
