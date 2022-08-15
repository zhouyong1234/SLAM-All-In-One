/*
 * This RansacPoseEstimation implementation was written based on pcl::SampleConsensusPrerejective
 *
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef HDL_GLOBAL_LOCALIZATION_RANSAC_POSE_ESTIMATION_HPP
#define HDL_GLOBAL_LOCALIZATION_RANSAC_POSE_ESTIMATION_HPP

#include <atomic>
#include <vector>
#include <random>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <hdl_global_localization/global_localization_results.hpp>
#include <hdl_global_localization/ransac/voxelset.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater.hpp>

namespace hdl_global_localization {

template <typename FeatureT>
class RansacPoseEstimation {
public:
  RansacPoseEstimation(ros::NodeHandle& private_nh);

  void set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, typename pcl::PointCloud<FeatureT>::ConstPtr target_features);
  void set_source(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, typename pcl::PointCloud<FeatureT>::ConstPtr source_features);

  GlobalLocalizationResults estimate();

private:
  void select_samples(std::mt19937& mt, const std::vector<std::vector<int>>& similar_features, std::vector<int>& samples, std::vector<int>& correspondences) const;

private:
  ros::NodeHandle& private_nh;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target;
  typename pcl::PointCloud<FeatureT>::ConstPtr target_features;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source;
  typename pcl::PointCloud<FeatureT>::ConstPtr source_features;

  typename pcl::KdTreeFLANN<FeatureT>::Ptr feature_tree;
  std::unique_ptr<MatchingCostEvaluater> evaluater;
};

}  // namespace hdl_global_localization

#endif