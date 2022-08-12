#pragma once 
#include "Config.hpp"
namespace vio{
class Camera {
 public:
  
  /** \brief Construct Function
   * @param cfg  ---  config object
   */ 
  Camera(const Config* cfg):
  Width(cfg->camParam_.width),
  Height(cfg->camParam_.height) {
    is_fisheye_ = cfg->camParam_.is_fisheye;
    cfg->camParam_.K.convertTo(K_,CV_32F);
    cfg->camParam_.D.convertTo(D_,CV_32F);
    fx_ = K_.at<float>(0,0);
    fy_ = K_.at<float>(1,1);
    cx_ = K_.at<float>(0,2);
    cy_ = K_.at<float>(1,2);
  };
  /** \brief Construct Function
   * @param K         ---   intrinisic matrix
   * @param D         ---   distortion matrix
   * @param isFishEye ---   fisheye flg
   */ 
  Camera(cv::Mat K,cv::Mat D,bool isFishEye,int w,int h):
  Width(w),
  Height(h) {
    is_fisheye_ = isFishEye;
    K.convertTo(K_,CV_32F);
    D.convertTo(D_,CV_32F);
    fx_ = K_.at<float>(0,0);
    fy_ = K_.at<float>(1,1);
    cx_ = K_.at<float>(0,2);
    cy_ = K_.at<float>(1,2);
  }

  /** \brief disable distortion for test 
   */ 
  void setNoDistortion() {
    D_ = cv::Mat::zeros(1,4,CV_64F);
  }

  /** \brief project 3D points into image plane
   * @param pts3D  --- array of 3D point in the world
   * @param uv     --- array of output pixel in the frame
   * @param CrW    --- rotation vector from world to Camera 
   * @param CtW    --- translate vector from world to Camera
   */ 
  void project(const std::vector<cv::Point3f> &pts3D,std::vector<cv::Point2f> &uv,cv::Mat CrW,cv::Mat CtW) {
    if (pts3D.empty()) {
      return;
    }
    if (is_fisheye_) {
      cv::fisheye::projectPoints(pts3D,uv,CrW,CtW,K_,D_);
    } else {
      cv::projectPoints(pts3D,CrW,CtW,K_,D_,uv);
    }
  } 

  /** \brief project 3D point into image plane
   * @param pts3D  --- array of 3D point in the world
   * @param CrW    --- rotation vector from world to Camera 
   * @param CtW    --- translate vector from world to Camera
   * @return pixel coordinate 
   */ 
  cv::Point2f project(const cv::Point3f &pts3D,
                      cv::Mat CrW = cv::Mat::zeros(cv::Size(1,3),CV_32F),
                      cv::Mat CtW = cv::Mat::zeros(cv::Size(1,3),CV_32F)) {
    cv::Mat uv;
    cv::Mat mat(1,3,CV_32F);
    mat.at<float>(0,0) = pts3D.x;
    mat.at<float>(0,1) = pts3D.y;
    mat.at<float>(0,2) = pts3D.z;
    mat = mat.reshape(3);
    if (is_fisheye_) {
      cv::fisheye::projectPoints(mat,uv,CrW,CtW,K_,D_);
    } else {
      cv::projectPoints(mat,CrW,CtW,K_,D_,uv);
    }
    uv.reshape(1);
    return cv::Point2f(uv.at<float>(0,0),uv.at<float>(0,1));
  } 


  /** \brief normalize feature from image plane into normalized plane
   * @param uv   --- pixel
   * @return return point in the normalized plane
   */ 
  cv::Point2f normalized(const cv::Point2f &uv) const{
    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = uv.x;
    mat.at<float>(0, 1) = uv.y;
    mat = mat.reshape(2); // Nx1, 2-channel
    // Undistort it!
    if (is_fisheye_) {
      cv::fisheye::undistortPoints(mat,mat,K_,D_);
    } else {
      cv::undistortPoints(mat, mat, K_, D_);
    }
    // Construct our return vector
    cv::Point2f pt_out;
    mat = mat.reshape(1); // Nx2, 1-channel
    pt_out.x = mat.at<float>(0, 0);
    pt_out.y = mat.at<float>(0, 1);
    return pt_out;
  };

  /** \brief get rectify map  
  */
  void getRectifyMap(cv::Mat &map1,cv::Mat &map2,double scale = 1.0) {
    if (is_fisheye_) {
      cv::Mat new_M;
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_, D_, cv::Size(Width,Height), 
                          cv::noArray(), new_M, 1.0, cv::Size(Width * scale,Height * scale), 1);
      cv::fisheye::initUndistortRectifyMap(K_,D_,cv::noArray(), new_M,cv::Size(Width * scale, Height * scale) ,CV_32FC1, map1, map2);
    } else {
      cv::Mat new_M = cv::getOptimalNewCameraMatrix(K_, D_, cv::Size(Width,Height), 1, cv::Size(Width * scale,Height * scale));
      cv::initUndistortRectifyMap(K_, D_, cv::Mat(), new_M, cv::Size(Width * scale, Height * scale), CV_32FC1,map1, map2);
    }
  }

  cv::Mat undistImage(const cv::Mat& image,const cv::Mat &map1,const cv::Mat &map2) {
    cv::Mat undistortImg;
    cv::remap(image, undistortImg, map1, map2, cv::INTER_LINEAR);
    return undistortImg;
  }

  /** \brief return intrinsic matrix
   */ 
  cv::Mat K() const {
    return K_;
  }

  /** \brief check pixel coordinate is in the camera
   */ 
  inline bool isInFrame(const cv::Point2f& uv) const{
    int x = cvRound(uv.x);
    int y = cvRound(uv.y);
    return (x > 2 && x < (Width - 2) && y > 2 && y < (Height - 2));
  }
  /** \brief get focal length in the x-axis
   */ 
  inline float fx() const {
    return fx_;
  }

  /** \brief get focal length in the y-axis
   */ 
  inline float fy() const {
    return fy_;
  }

  /** \brief get origin point x coordinate in the image plane
   */
  inline float cx() const {
    return cx_;
  }

  /** \brief get origin point y coordinate in the image plane
   */ 
  inline float cy() const {
    return cy_;
  }

  /** \brief get image width
   */ 
  inline float width() const {
    return Width;
  }
  
  /** \brief get image height 
   */ 
  inline float height() const {
    return Height;
  }
 private:
  bool is_fisheye_;
  cv::Mat K_;
  cv::Mat D_;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  const int Width,Height;
 
};
typedef std::shared_ptr<Camera> CameraPtr;
}