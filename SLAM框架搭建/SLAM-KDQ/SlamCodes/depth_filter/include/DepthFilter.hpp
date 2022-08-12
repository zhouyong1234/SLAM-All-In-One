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

#ifndef SVO_DEPTH_FILTER_H_
#define SVO_DEPTH_FILTER_H_

#include <iostream>
#include <queue>
#include <map>
#include <list>
#include <thread>
#include <condition_variable>
#include <Eigen/Dense>

namespace depth_filter {

class Corner;
typedef std::map<int, Corner> Corners;

///SeedPoint contains points in world frame for showing uncertainty of seed
struct SeedPoint {
  /** Construction
   * @param mean --- seed coordinate with mean depth
   * @param min  --- seed coordinate with min depth
   * @param max  --- seed coordinate with max depth
   */
  SeedPoint(Eigen::Vector3d mean, Eigen::Vector3d min, Eigen::Vector3d max) :
    meanPtsInWorld(mean),
    minPtsInWorld(min),
    maxPtsInWorld(max) {
  }
  Eigen::Vector3d meanPtsInWorld; //<! seed coordinate with mean depth
  Eigen::Vector3d minPtsInWorld;  //<! seed coordinate with min depth
  Eigen::Vector3d maxPtsInWorld;  //<! seed coordinate with max depth
};

/// Corner in images includes pixel coordinate and bearing vector etc
class Corner {
 public:
  //Default construction
  Corner() = default;
  /** Construction
   * @param id --- corner id
   * @param depth --- depth of corner in world frame
   * @param unitBearingVector --- point coordinate in normalized plane
   */
  Corner(int id, double depth, Eigen::Vector3d unitBearingVector) :
    id_(id),
    depth_(depth),
    unitBearingVector_(unitBearingVector) {
  }
  /**  Copy
   * @param c  --- corner object
   */
  Corner(const Corner &c) {
    id_ = c.id_;
    depth_ = c.depth_;
    unitBearingVector_ = c.unitBearingVector_;
  }

  int id_;                             //!< Id of corner
  double depth_;                       //!< Depth of corner vector
  Eigen::Vector3d unitBearingVector_;  //!< Unit-bearing vector of the feature.
};

/// Frame includes pose and features
class Frame {
 public:

  /** Construction
   * @param Tcw  --- Translation from world to camera
   */
  Frame(Eigen::Isometry3d Tcw,Eigen::Matrix3d transNoise,double pixelNoise) : Tcw_(Tcw), transNoise_(transNoise), pixelNoise_(pixelNoise) {};

  /** Get corner through its id
   *
   * @param id  --- corner's id
   * @return a pointer points corner object
   */
  Corner *getCorner(int id) {
    if (corners_.count(id)) {
      return &corners_[id];
    } else {
      return nullptr;
    }
  }

  /** Get corners in frame
   *
   * @return reference of corners
   */
  Corners &getCorners() {
    return corners_;
  }

  /** Insert a corner into corners
   *
   * @param c --- corner object
   */
  void insertCorner(Corner &c) {
    if (!corners_.count(c.id_)) {
      corners_[c.id_] = c;
    }
  }
  Eigen::Isometry3d Tcw_; //!< Translation from world to camera
  Eigen::Matrix3d transNoise_;
  double pixelNoise_;
 private:
  Corners corners_;       //!< Contains all corners of frame should view
};

/// A seed is a probabilistic depth estimate for a single pixel.
struct Seed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Corner c_;                     //!< Corner in the keyframe for which the depth should be computed.
  float a_;                     //!< a of Beta distribution: When high, probability of inlier is large.
  float b_;                     //!< b of Beta distribution: When high, probability of outlier is large.
  float mu_;                    //!< Mean of normal distribution.
  float depthRange_;               //!< Max range of the possible depth.
  float sigma2_;                //!< Variance of normal distribution.
  Eigen::Isometry3d Tcw_;   //!< Translation from world to reference camera frame
  int lostCount_;              //!< Lost count for corner not observed
  /* Construct
   * @param cor  --- corner object
   * @param Tcw  --- Translation from world to corner's reference camera
   * @param depth_mean  --- mean value of corner depth
   * @param depth_min   --- min value of corner depth
   */
  Seed(Corner &c, Eigen::Isometry3d Tcw, float depthMean, float depthMin,float a = 10.,float b = 10.);

  Seed(float depthMean,float depthMin,float a = 10,float b = 10.);
};

/// Depth filter implements the Bayesian Update proposed in:
/// "Video-based, Real-Time Multi View Stereo" by G. Vogiatzis and C. Hernández.
/// In Image and Vision Computing, 29(7):434-441, 2011.
///
/// The class uses a callback mechanism such that it can be used also by other
/// algorithms than nslam and for simplified testing.
class DepthFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Frame> FramePtr;
  typedef std::unique_lock<std::mutex> lock_t;
  typedef std::function<void(const std::vector<SeedPoint> &)> callback_t;

  /// Depth-filter config parameters
  struct Options {
    bool check_ftr_angle;                       //!< gradient features are only updated if the epipolar line is orthogonal to the gradient.
    bool epi_search_1d;                         //!< restrict Gauss Newton in the epipolar search to the epipolar line.
    bool verbose;                               //!< display output.
    bool use_photometric_disparity_error;       //!< use photometric disparity error instead of 1px error in tau computation.
    int max_lost_fs;                              //!< maximum number of keyframes for which we maintain seeds.
    double sigma_i_sq;                          //!< image noise.
    double seed_convergence_sigma2_thresh;      //!< threshold on depth uncertainty for convergence.
    Options()
      : check_ftr_angle(false),
        epi_search_1d(false),
        verbose(false),
        use_photometric_disparity_error(false),
        max_lost_fs(10),
        sigma_i_sq(5e-4),
        seed_convergence_sigma2_thresh(200.0) {}
  } options_;

  DepthFilter() = default;

  /** Construction
   *
   * @param K       --- camera intrinsic matrix
   * @param width   --- width of image
   * @param height  --- height of image
   * @param seedCallback --- callback function
   */
  DepthFilter(Eigen::Matrix3d K,int width,int height,callback_t seedCallback);

  /**
   *
   */
  virtual ~DepthFilter();

  /// Start this thread when seed updating should be in a parallel thread.
  void startThread();

  /// Stop the parallel thread that is running.
  void stopThread();

  /// Add frame to the queue to be processed.
  void addFrame(FramePtr frame);

  /// Add new keyframe to the queue
  void addKeyframe(FramePtr frame, double depth_mean, double depth_min);

  /// If the map is reset, call this function such that we don't have pointers
  /// to old frames.
  void reset();

  /// Return a reference to the seeds. This is NOT THREAD SAFE!
  std::list<Seed> &getSeeds() { return seeds_; }

  /// Bayes update of the seed, x is the measurement, tau2 the measurement uncertainty
  static void updateSeed(
    const float x,
    const float tau2,
    Seed *seed);

  /// Compute the uncertainty of the measurement.
  //KDQ： 特征点在参考帧因为像素误差可能导致的和真值的深度误差
  static double computeTau(
    const Eigen::Isometry3d &T_ref_cur,
    const Eigen::Vector3d &f,
    const double z,
    const double px_error_angle);

 protected:
  callback_t seedCallback_;
  std::list<Seed> seeds_;
  std::map<int, bool> seedIds_;
  std::mutex seedsMut_;
  bool seedsUpdatingHalt_;            //!< Set this value to true when seeds updating should be interrupted.
  std::thread *thread_;
  std::queue<FramePtr> frameQueue_;
  std::mutex frameQueueMut_;
  std::condition_variable frameQueueCond_;
  FramePtr newKeyframe_;
  bool newKeyframeSet_;               //!< Do we have a new keyframe to process?.
  double newKeyframeMinDepth_;       //!< Minimum depth in the new keyframe. Used for range in new seeds.
  double newKeyframeMeanDepth_;      //!< Maximum depth in the new keyframe. Used for range in new seeds.
  Eigen::Matrix3d K_;
  int width_;
  int height_;
  bool stopThreadFlg_;
  /// Initialize new seeds from a frame.
  void initializeSeeds(FramePtr frame);

  /// Update all seeds with a new measurement frame.
  virtual void updateSeeds(FramePtr frame);

  /// When a new keyframe arrives, the frame queue should be cleared.
  void clearFrameQueue();

  /// A thread that is continuously updating the seeds.
  void updateSeedsLoop();

  bool depthFromTriangulation(
    const Eigen::Isometry3d &T_search_ref,
    const Eigen::Vector3d &f_ref,
    const Eigen::Vector3d &f_cur,
    const Eigen::Matrix3d &transNoise,
    double &depth,
    double &depthStdErr);

  inline bool isInFrame(Eigen::Vector2i uv) {
    return uv.x() > 1 && uv.x() < width_ && uv.y() > 1 && uv.y() < height_;
  }

  inline Eigen::Vector2d f2c(Eigen::Vector3d f) {
    Eigen::Vector3d nrVector(f.x()/f.z(),f.y()/f.z(),1.);
    Eigen::Vector3d uv = K_ * nrVector;
    return Eigen::Vector2d(uv.x(), uv.y());
  }

};

} // namespace svo

#endif // SVO_DEPTH_FILTER_H_
