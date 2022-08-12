#pragma once 
#include <memory>
#include <iostream>
#include <map>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "Camera.hpp"


namespace vio{

class Frame {
 public:

  /** \brief construct 
   */ 
  Frame(const CameraPtr cam):cam_(cam) {
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    setPoseInWorld(R,t);
    assert(cam_ != nullptr);
  };
  /** \brief construct function
   * @param timestamp --- timestamp of current frame
   * @param img       --- image data 
   */ 
  Frame(double timestamp,cv::Mat& img,const CameraPtr cam): timestamp_(timestamp),cam_(cam) {
    image_ = img.clone();
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    setPoseInWorld(R,t);
    assert(cam_ != nullptr);
  };

  /** \brief construct function
   * @param timestamp --- timestamp of current frame 
   * @param corners   --- corners map 
   */ 
  Frame(double timestamp,std::map<uint64_t,cv::Point2f>& corners,const CameraPtr cam):
  timestamp_(timestamp),corners_(corners),cam_(cam) {
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    setPoseInWorld(R,t);
    assert(cam_ != nullptr);
  }
  
  /** \brief initializing with frame object
   */ 
  Frame(const Frame& frame):cam_(frame.cam_) {
    timestamp_ = frame.timestamp_;
    corners_ = frame.getCorners();
    image_ = frame.image_.clone();
    frame.getPoseInWorld(Rwc_,WtC_);
    assert(cam_ != nullptr);
  }

  /** \brief unconstruct function 
   * 
   */ 
  ~Frame() {
    //corners_.clear();//自动调用析构函数
    //image_.release();//CV::Mat会自动调用析构函数
  }

  /** \brief set frame pose in the world frame
   * @param Rwc  ---  rotation matrix from frame to world
   * @param WtC  ---  translate matrix from frame to world 
   */ 
  void setPoseInWorld(cv::Mat Rwc,cv::Mat WtC) {
    Rwc.convertTo(Rwc_,CV_64F);
    WtC.convertTo(WtC_,CV_64F);
  }

  /** \brief get pose from frame to world
   * @param Rwc   --- rotation matrix from frame to world 
   * @param WtC   --- translate vector from frame to world
   */ 
  void getPoseInWorld(cv::Mat& Rwc,cv::Mat& WtC) const{
    Rwc_.copyTo(Rwc);
    WtC_.copyTo(WtC);
  }

  /** \brief get pose from world to frame
   * @param Rcw   --- rotation matrix from world to frame 
   * @param CtW   --- translate vector from world to frame
   */ 
  void getInversePose(cv::Mat& Rcw,cv::Mat& CtW) const{
    Rcw = Rwc_.t();
    CtW = -Rcw * WtC_;
  }
  
  /** \brief  set corners map
   * @param corners  --- corners map
   */ 
  void setCorners(std::map<uint64_t,cv::Point2f>& corners) {
    corners_ = corners;
  };

  /** \brief get corners map
   * @return corners map in the this frame
   */ 
  const std::map<uint64_t,cv::Point2f> &getCorners() const{
    return corners_;
  }

  /** \brief get corners map
   * @return corners map in the this frame
   */ 
  std::map<uint64_t,cv::Point2f> & getCorners() {
    return corners_;
  }

  /** \brief get corners' size
   * @return
   */
  int getCornerSize() const {
    return corners_.size();
  }
  /** \brief get normalized coordinate of id-corner
   *
   * @param id --- id of corner
   * @param normalizedPt --- normalized coordinate of id-corner
   * @return
   */
  bool getNormalizedUV(uint64_t id,cv::Point2f& normalizedPt) {
    bool successFlg = true;
    if (normalizedUV_.count(id)) {
      normalizedPt = normalizedUV_[id];
    } else if (corners_.count(id)){
      assert(cam_ != nullptr);
      normalizedUV_[id] = cam_->normalized(corners_[id]);
      normalizedPt = normalizedUV_[id];
    } else {
      successFlg = false;
    }
    return successFlg;
  }

  /** \brief get corners vector 
   * @param idx     ---  vector of feature id
   * @param corners ---  vector of feature uv corresponding id
   */ 
  void getCornerVector(std::vector<uint64_t> &idx,std::vector<cv::Point2f> &corners) {
    for (auto it = corners_.begin(); it != corners_.end(); it++) {
      idx.push_back(it->first);
      corners.push_back(it->second);
    }
  }

  /** \brief set corners map 
   * @param idx     ---  vector of feature id
   * @param corners ---  vector of feature uv corresponding id
   */ 
  void setCornerMap(const std::vector<uint64_t> &idx,const std::vector<cv::Point2f> &corners) {
    for (size_t i = 0; i < idx.size(); i++) {
      corners_[idx[i]] = corners[i];
    }
  }

  /** \brief get corner pixel coordinate corresponding id 
   * @param id --- corner id
   * @param uv --- pixel coordinate reference 
   * @return return false if no this id 
   */   
  bool getCornerUV(uint64_t id,cv::Point2f& uv) const{
    if (!corners_.count(id)) {
      return false;
    }
    uv = corners_.at(id);
    return true;
  }

  /** \brief get all both matched features with reference frame
   * @param  refFrame  --- reference frame 
   * @param matchedId  --- matched id vector
   * @param refCorners --- matched pixel vector in the reference frame
   * @param curCorners --- matched pixel vector in the current frame
   */ 
  void getMatchedFeatures(const Frame* refFrame,
                          std::vector<uint64_t> &matchedId,
                          std::vector<cv::Point2f> &refCorners,
                          std::vector<cv::Point2f> &curCorners,
                          float &averParallex) {
    averParallex = 0;
    float sumParallex = 0;
    const std::map<uint64_t,cv::Point2f> refFeats = refFrame->getCorners();
    for (auto it = refFeats.begin();it != refFeats.end();it++) {
      if (corners_.count(it->first)) {
        matchedId.push_back(it->first);
        refCorners.push_back(it->second);
        curCorners.push_back(corners_[it->first]);
        sumParallex += cv::norm(refCorners.back() - curCorners.back());
      }
    }
    if (curCorners.size() > 0) {
      averParallex = sumParallex / curCorners.size();
    }
  }

  /** \brief get matched feature size
   *
   * @param frame  --- frame pointer
   * @return
   */
  int getMatchedFeatureSize(const Frame* frame) {
    int matchedSize = 0;
    const std::map<uint64_t,cv::Point2f>& refFeats = frame->getCorners();
    for (auto it = refFeats.begin();it != refFeats.end();it++) {
      if (corners_.count(it->first)) {
        matchedSize++;
      }
    }
    return matchedSize;
  }

  /** \brief get matched feature size
 *
 * @param frame  --- frame pointer
 * @return
 */
  int getMatchedFeatureSize(const Frame* frame,float& averParallex) {
    int matchedSize = 0;
    float sumParallex = 0;
    const std::map<uint64_t,cv::Point2f>& refFeats = frame->getCorners();
    for (auto it = refFeats.begin();it != refFeats.end();it++) {
      if (corners_.count(it->first)) {
        matchedSize++;
        sumParallex += cv::norm(it->second - corners_[it->first]);
      }
    }
    if (matchedSize > 0) {
      averParallex = sumParallex / matchedSize;
    }
    return matchedSize;
  }


  /** \brief display all feature in current frame
   * @param scale --- image scale, 1 - means raw size; 2 - means twice size bigger than raw size
   */
  void imshowFeatures(uint scale = 1) {
    if (image_.empty()) {
      return;
    }
    cv::Mat colorImage;
    cv::cvtColor(image_,colorImage,cv::COLOR_GRAY2BGR);
    for (auto it = corners_.begin(); it != corners_.end(); it++) {
      cv::Point2f uv = it->second;
      cv::circle(colorImage,uv,2,cv::Scalar(0,255,0));
      cv::putText(colorImage,std::to_string(it->first),uv,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
    }
    cv::resize(colorImage,colorImage,cv::Size(colorImage.cols*scale,colorImage.rows*scale));
    cv::imshow("test_track",colorImage);
    cv::waitKey(1);
  }

  /** \brief get rotation matrix
   */ 
  inline cv::Mat Rwc() const {
    return Rwc_;
  }

  inline Eigen::Matrix3d ERwc() const {
    Eigen::Matrix3d eRwc;
    cv::cv2eigen(Rwc_,eRwc);
    return eRwc;
  }

  /** \brief get translate part
   */ 
  inline cv::Mat WtC() const {
    return WtC_;
  }

  inline Eigen::Vector3d EWtC() const {
    Eigen::Vector3d eWtc;
    cv::cv2eigen(WtC_,eWtc);
    return eWtc;
  }

  inline Eigen::Isometry3d ETwc() const {
    Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
    Twc.rotate(ERwc());
    Twc.pretranslate(EWtC());
    return Twc;
  }

  cv::Mat image_;
  double timestamp_;
  const CameraPtr cam_;

 private:
  cv::Mat Rwc_,WtC_; // rotation matrix and translate vector from camera to world
  std::map<uint64_t,cv::Point2f> corners_;
  std::map<uint64_t,cv::Point2f> normalizedUV_;
};

typedef std::shared_ptr<Frame> FramePtr;

}