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

#include "DepthFilter.hpp"
#include <algorithm>
#include <boost/math/distributions/normal.hpp>

namespace depth_filter {

Seed::Seed(Corner &c,Eigen::Isometry3d Tcw,float depthMean,float depthMin,float a,float b):
    c_(c),
    Tcw_(Tcw),
    a_(a),
    b_(b),
    mu_(1.0 / depthMean),
    depthRange_(1.0 / depthMin), // depthRange = (1/minDepth - 1/maxDepth) ~= 1/minDepth
    sigma2_(depthRange_ * depthRange_ / 36.),
    lostCount_(0) {
}


Seed::Seed(float depthMean,float depthMin,float a,float b):
    a_(a),
    b_(b),
    mu_(1.0 / depthMean),
    depthRange_(1.0 / depthMin), // depthRange = (1/minDepth - 1/maxDepth) ~= 1/minDepth
    sigma2_(depthRange_ * depthRange_ / 36.),
    lostCount_(0) {
  Tcw_.setIdentity();
}

DepthFilter::DepthFilter(Eigen::Matrix3d K,int width,int height,callback_t seedCallback) :
    seedCallback_(seedCallback),
    seedsUpdatingHalt_(false),
    thread_(NULL),
    newKeyframeSet_(false),
    newKeyframeMinDepth_(0.0),
    newKeyframeMeanDepth_(0.0),
    K_(K),
    width_(width),
    height_(height),
    stopThreadFlg_(false) {
}
DepthFilter::~DepthFilter() {
  stopThread();
  printf("DepthFilter destructed.");
}

void DepthFilter::startThread() {
  thread_ = new std::thread(&DepthFilter::updateSeedsLoop, this);
}

void DepthFilter::stopThread() {
  printf("DepthFilter stop thread invoked.");
  if (thread_ != NULL) {
     printf("DepthFilter interrupt and join thread... ");
     seedsUpdatingHalt_ = true;
     if (thread_->joinable()) {
       stopThreadFlg_ = true;
       thread_->join();
     }
     thread_ = NULL;
  }
}

void DepthFilter::addFrame(FramePtr frame) {
  if (thread_ != NULL) {
    {
      lock_t lock(frameQueueMut_);
      if (frameQueue_.size() > 2)
        frameQueue_.pop();
      frameQueue_.push(frame);
    }
    seedsUpdatingHalt_ = false;
    frameQueueCond_.notify_one();
  } else {
    updateSeeds(frame);
  }
}

void DepthFilter::addKeyframe(FramePtr frame, double depth_mean, double depth_min) {
  newKeyframeMinDepth_ = depth_min;
  newKeyframeMeanDepth_ = depth_mean;
  if (thread_ != NULL) {
    newKeyframe_ = frame;
    newKeyframeSet_ = true;
    seedsUpdatingHalt_ = true;
    frameQueueCond_.notify_one();
  } else {
    initializeSeeds(frame);
  }
}

void DepthFilter::initializeSeeds(FramePtr frame) {
  // initialize a seed for every new feature
  seedsUpdatingHalt_ = true;
  lock_t lock(seedsMut_); // by locking the updateSeeds function stops
  Corners &feats = frame->getCorners();
  for (auto it = feats.begin(); it != feats.end(); it++) {
    if (!seedIds_.count(it->second.id_)) {
      seeds_.push_back(Seed(*(&(it->second)), frame->Tcw_, newKeyframeMeanDepth_, newKeyframeMinDepth_));
      seedIds_[it->second.id_] = true;
    }
  }
  seedsUpdatingHalt_ = false;
}

void DepthFilter::reset() {
  seedsUpdatingHalt_ = true;
  {
    lock_t lock(seedsMut_);
    seeds_.clear();
    seedIds_.clear();
  }
  lock_t lock();
  while (!frameQueue_.empty())
    frameQueue_.pop();
  seedsUpdatingHalt_ = false;

  if (options_.verbose)
    printf("DepthFilter: RESET.");
}

void DepthFilter::updateSeedsLoop() {
  while (!stopThreadFlg_) {
    FramePtr frame;
    {
      lock_t lock(frameQueueMut_);
      while (frameQueue_.empty() && newKeyframeSet_ == false)
        frameQueueCond_.wait(lock);
      if (newKeyframeSet_) {
        newKeyframeSet_ = false;
        seedsUpdatingHalt_ = false;
        clearFrameQueue();
        frame = newKeyframe_;
      } else {
        frame = frameQueue_.front();
        frameQueue_.pop();
      }
    }
    updateSeeds(frame);
    initializeSeeds(frame);
  }
}

bool DepthFilter::depthFromTriangulation(
    const Eigen::Isometry3d &T_search_ref,
    const Eigen::Vector3d &f_ref,
    const Eigen::Vector3d &f_cur,
    const Eigen::Matrix3d &transNoise,
    double &depth,
    double &depthStdErr) {
  Eigen::Matrix<double, 3, 2> A;
  A << T_search_ref.rotation() * f_ref, f_cur;
  const Eigen::Matrix2d AtA = A.transpose() * A;
  //如果当前帧和
  if (AtA.determinant() < 0.000001)
    return false;
  Eigen::Matrix<double,2,3> B = -AtA.inverse() * A.transpose();
  const Eigen::Vector2d depth2 = B * T_search_ref.translation();
  Eigen::Matrix<double,2,2> depthNoise = B * transNoise * B.transpose();
  depthStdErr = sqrt(depthNoise(0,0));
  depth = fabs(depth2[0]);
  return true;
}

void DepthFilter::updateSeeds(FramePtr frame) {
  // update only a limited number of seeds, because we don't have time to do it
  // for all the seeds in every frame!
  lock_t lock(seedsMut_);
  std::list<Seed>::iterator it = seeds_.begin();
  std::list<Seed>::iterator itEnd = seeds_.end();
  const double focal_length = K_(0,0);
  double px_error_angle = atan(frame->pixelNoise_ / (2.0 * focal_length)) * 2.0; // law of chord (sehnensatz)
  std::vector<SeedPoint> clouds;
  while (it != seeds_.end()) {
    // set this value true when seeds updating should be interrupted
    if (seedsUpdatingHalt_)
      return;
    //如果这个点在当前帧没有出现，且丢失次数超过阈值则剔除
    if (it->lostCount_ > options_.max_lost_fs) {
      if (seedIds_.count(it->c_.id_)) {
        seedIds_.erase(it->c_.id_);
      }
      printf("Delete %d for lost Count %d \n",it->c_.id_,it->lostCount_);
      it = seeds_.erase(it);
      continue;
    }
    Corner *curCor = frame->getCorner(it->c_.id_);
    if (curCor == nullptr) {
      it->lostCount_++;
      ++it;
      continue;
    }
    // check if point is visible in the current image
    Eigen::Isometry3d T_ref_cur = it->Tcw_ * frame->Tcw_.inverse();

    // we are using inverse depth coordinates
    double z,z_std_err;
    //对于很远的点，如果水平运动很小的话，导致bearingVector几乎没有变，那么在解算深度的时候只有一个方程有效，最终深度受到噪音的影响变大
    if (!depthFromTriangulation(T_ref_cur.inverse(), it->c_.unitBearingVector_, curCor->unitBearingVector_,frame->transNoise_,z,z_std_err)) {
      it->b_++; // increase outlier probability when no match was found
      ++it;
      printf("Triangulate failed!\n");
      continue;
    }
    // 这里面有个问题，如果一开始初始化的深度极其不准，某些情况下导致该点重投影回当前帧根本就看不到?
    const Eigen::Vector3d xyz_f(T_ref_cur.inverse() * (z * it->c_.unitBearingVector_));
    if (!isInFrame(f2c(xyz_f).cast<int>())) {
      it->lostCount_++;
      it->b_++;
      ++it; // point does not project in image
      printf("Not in reference camera!\n");
      continue;
    }
    // 计算因为特征点像素误差导致的深度误差
    double tau = computeTau(T_ref_cur, it->c_.unitBearingVector_, z, px_error_angle);
    double tau2 = computeTau(T_ref_cur, it->c_.unitBearingVector_, z + z_std_err, px_error_angle);
    double tau_inverse = 0.5 * (1.0 / std::max(0.0000001, z - tau) - 1.0 / (z + tau));
    printf("depth : %f depthErr : %f and tau:  %f, tau2: %f\n",z,z_std_err,tau,tau2);
    // update the estimate
    updateSeed(1. / z, tau_inverse * tau_inverse, &*it);
    float z_inv_min = it->mu_ + sqrt(it->sigma2_);
    float z_inv_max = std::max(it->mu_ - sqrt(it->sigma2_), 0.00000001f);
    // if the seed has converged, we initialize a new candidate point and remove the seed
    if (!isnan(z_inv_min) || sqrt(it->sigma2_) < it->depthRange_ / options_.seed_convergence_sigma2_thresh) {
      //assert(it->c.point == NULL); // TODO this should not happen anymore
      SeedPoint xyz_world(it->Tcw_.inverse() * (it->c_.unitBearingVector_ * (1.0 / it->mu_)),
                          it->Tcw_.inverse() * (it->c_.unitBearingVector_ * (1.0 / z_inv_min)),
                          it->Tcw_.inverse() * (it->c_.unitBearingVector_ * (1.0 / z_inv_max)));
      clouds.push_back(xyz_world);
      ++it;
    } else if (isnan(z_inv_min)) {
      if (seedIds_.count(it->c_.id_)) {
        seedIds_.erase(it->c_.id_);
      }
      it = seeds_.erase(it);
    } else {
      ++it;
    }
  }
  seedCallback_(clouds);
}

void DepthFilter::clearFrameQueue() {
  while (!frameQueue_.empty())
    frameQueue_.pop();
}

void DepthFilter::updateSeed(const float x, const float tau2, Seed *seed) {
  float norm_scale = sqrt(seed->sigma2_ + tau2);
  if (std::isnan(norm_scale))
    return;
  boost::math::normal_distribution<float> nd(seed->mu_, norm_scale);
  float s2 = 1. / (1. / seed->sigma2_ + 1. / tau2);
  float m = s2 * (seed->mu_ / seed->sigma2_ + x / tau2);
  float C1 = seed->a_ / (seed->a_ + seed->b_) * boost::math::pdf(nd, x);
  float C2 = seed->b_ / (seed->a_ + seed->b_) * 1. / seed->depthRange_;
  float normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  float f = C1 * (seed->a_ + 1.) / (seed->a_ + seed->b_ + 1.) + C2 * seed->a_ / (seed->a_ + seed->b_ + 1.);
  float e = C1 * (seed->a_ + 1.) * (seed->a_ + 2.) / ((seed->a_ + seed->b_ + 1.) * (seed->a_ + seed->b_ + 2.))
      + C2 * seed->a_ * (seed->a_ + 1.0f) / ((seed->a_ + seed->b_ + 1.0f) * (seed->a_ + seed->b_ + 2.0f));

  // update parameters
  float mu_new = C1 * m + C2 * seed->mu_;
  seed->sigma2_ = C1 * (s2 + m * m) + C2 * (seed->sigma2_ + seed->mu_ * seed->mu_) - mu_new * mu_new;
  seed->mu_ = mu_new;
  seed->a_ = (e - f) / (f - e / f);
  seed->b_ = seed->a_ * (1.0f - f) / f;
#ifdef  VERBOOSE
  float inlierP = seed->a_ / (seed->a_ + seed->b_);
  float stdErr = sqrt(seed->sigma2_);
  printf("%d,%f,%f,%f,%f,%f,%f,%e\n",seed->c_.id_,stdErr,1./x,inlierP,seed->a_,seed->b_,1./seed->mu_,seed->sigma2_);
#endif
}

double DepthFilter::computeTau(
    const Eigen::Isometry3d &T_ref_cur,
    const Eigen::Vector3d &f,
    const double z,
    const double px_error_angle) {
  Eigen::Vector3d t(T_ref_cur.translation());
  Eigen::Vector3d a = f * z - t;
  double t_norm = t.norm();
  double a_norm = a.norm();
  double alpha = acos(f.dot(t) / t_norm); // dot product
  double beta = acos(a.dot(-t) / (t_norm * a_norm)); // dot product
  double beta_plus = beta + px_error_angle;
  double gamma_plus = M_PI - alpha - beta_plus; // triangle angles sum to PI
  double z_plus = t_norm * sin(beta_plus) / sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}

} // namespace svo
