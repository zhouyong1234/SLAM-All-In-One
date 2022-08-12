#pragma once
#include <opencv2/core.hpp>
#include "Config.hpp"
#include "Frame.hpp"
#include "Camera.hpp"
namespace vio{
class FeatureTracker{
 public:

  /** \brief construct
   */ 
  FeatureTracker(const Config* cfg);

  /** \brief detect new features and track last features in current frame
   * @param refFrame  --- reference frame ptr
   * @param curFrame  --- current frame ptr
   */ 
  void detectAndTrackFeature(FramePtr refFrame,FramePtr curFrame,const cv::Mat &RcurLast);

  /** \brief
   *
   * @param img  --- image to display all features
   * @param imgScale --- image scale times than raw size
   */
  void showAllFeature(double timestamp,cv::Mat &img,uint8_t imgScale = 1);

  /** \brief reset feature track system through clear trackCount and allCorners
   *
   */
  void reset() {
    id_ = 0;
    trackCount_.clear();
    allCorners_.clear();
  }
  

 private:
  std::string moduleName_;
  const int MaxPointSize;
  const float QualityLevel;
  const float MinDist;
  const int TrackBack;
  const int PredictEnable;
  const int TrackBackPixelErr;
  const int PyramidLevel;
  const int CriterIterations;
  const double CriterEPS;
  const int ShowTrackFrames;
  const int ShowDebugInfos;
  Camera* cam_; 
  uint64_t id_;
  std::vector<uint64_t> trackCount_;
  std::list<std::map<uint64_t,cv::Point2f>> allCorners_;
  /** \brief remove bad data in the vector
   * @param status  ---  status vector 1 is good,0 needs remove
   * @param data    ---  vector data 
   */ 
  template <class T>
  inline void remove(const std::vector<uchar>& status,std::vector<T>& data) {
    assert(status.size() == data.size());
    size_t i = 0,j = 0;
    for (i = 0; i < status.size(); i++) {
      if (status[i]) {
        data[j] = data[i];
        j++;
      }
    } 
    data.resize(j);
  }

  /** \brief set mask at given features
   * @param img  ---  image size
   * @param feat ---  vector of featurs
   */ 
  cv::Mat setMask(const cv::Mat& img,std::vector<uint64_t>& idx,std::vector<cv::Point2f>& feat) {
    assert(trackCount_.size() == feat.size());
    cv::Mat mask(cv::Size(img.cols,img.rows),CV_8UC1,255);
    if (feat.size() < 2) {
      return mask;
    }
    std::vector<std::pair<uint64_t,std::pair<uint64_t,cv::Point2f>>> sortVec;
    for (size_t i = 0; i < idx.size(); i++) {
      sortVec.push_back(std::make_pair(trackCount_[i],std::make_pair(idx[i],feat[i])));
    }
    std::sort(sortVec.begin(), sortVec.end(), [](const std::pair<uint64_t, std::pair<uint64_t, cv::Point2f>> &a, 
      const std::pair<uint64_t, std::pair<uint64_t, cv::Point2f>> &b) {
      return a.first > b.first;
    });
    idx.clear();
    trackCount_.clear();
    feat.clear();
    for (size_t i = 0; i < sortVec.size(); i++) {
      if (mask.at<uchar>(sortVec[i].second.second) == 255) {
        cv::circle(mask,sortVec[i].second.second,MinDist,0,-1);
        idx.push_back(sortVec[i].second.first);
        feat.push_back(sortVec[i].second.second);
        trackCount_.push_back(sortVec[i].first);
      }
    }
    return mask;
  }

  std::vector<cv::Point2f> predictFeatures(const std::vector<cv::Point2f>& lastCorners,const cv::Mat& RcurLast);
};
}


