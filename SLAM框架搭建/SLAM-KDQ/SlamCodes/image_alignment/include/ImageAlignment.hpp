#include "iostream"
#include "opencv2/opencv.hpp"
#include "Camera.hpp"

class ImageAlignment {
 public:
  ImageAlignment(int motionModel,int iterationNum,double eps,int pydownLevel = 1,bool histgram = false):
    MotionModel_(motionModel),
    IterationNumber_(iterationNum),
    TerminationEps_(eps),
    Histgram_(histgram),
    PyrDownLevel_(pydownLevel){
    if (PyrDownLevel_ < 1) {
      PyrDownLevel_ = 1;
    }
    cam_ = nullptr;
  }

  ImageAlignment(Camera* cam,int motionModel,int iterationNum,double eps,int pydownLevel = 1,bool histgram = false):
    cam_(cam),
    MotionModel_(motionModel),
    IterationNumber_(iterationNum),
    TerminationEps_(eps),
    Histgram_(histgram),
    PyrDownLevel_(pydownLevel){
    if (PyrDownLevel_ < 1) {
      PyrDownLevel_ = 1;
    }
    if (cam_ != nullptr) {
      cam_->getRectifyMap(mapx_,mapy_,K_,1.0/(float)(PyrDownLevel_));
    }
  }

  cv::Mat Align(double timestamp,cv::Mat& templateIm,cv::Mat& targetIm,bool writeImage = false) {
    cv::Mat img1,img2;
    if (templateIm.size() != targetIm.size()) {
      return cv::Mat();
    }
    if (PyrDownLevel_ > 1 && cam_ == nullptr) {
      cv::resize(templateIm,img1,templateIm.size()/PyrDownLevel_,0,0,CV_INTER_AREA);
      cv::resize(targetIm,img2,targetIm.size()/PyrDownLevel_,0,0,CV_INTER_AREA);
    } else {
      if (cam_ != nullptr) {
        img1 = cam_->undistImage(templateIm,mapx_,mapy_);
        img2 = cam_->undistImage(targetIm,mapx_,mapy_);
      } else {
        img1 = templateIm;
        img2 = targetIm;
      }
    }
    cv::Mat warpMatrix;
    if (MotionModel_ > cv::MOTION_HOMOGRAPHY || MotionModel_ < cv::MOTION_TRANSLATION) {
      MotionModel_ = cv::MOTION_HOMOGRAPHY;
    }
    if (MotionModel_ == cv::MOTION_HOMOGRAPHY) {
      warpMatrix = cv::Mat::eye(3, 3, CV_32F);
    } else {
      warpMatrix = cv::Mat::eye(2, 3, CV_32F);
    }
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, IterationNumber_, TerminationEps_);
    cv::Mat im1 = img1.clone(),im2 = img2.clone();
    if (templateIm.channels() == 3) {
      cv::cvtColor(templateIm,im1,CV_BGR2GRAY);
    }
    if (targetIm.channels() == 3) {
      cv::cvtColor(targetIm,im2,CV_BGR2GRAY);
    }
    if (Histgram_) {
      cv::equalizeHist(im1,im1);
      cv::equalizeHist(im2,im2);
    }
    try {
      cv::findTransformECC(im1,im2,warpMatrix,MotionModel_,termCriteria);
    } catch (cv::Exception &e){
      std::cerr << e.msg << std::endl;
      return warpMatrix;
    }
#ifdef PC_VERSION
    cv::Mat im2_aligned;
    if (MotionModel_ != cv::MOTION_HOMOGRAPHY) {
      // Use warpAffine for Translation, Euclidean and Affine
      cv::warpAffine(im2, im2_aligned, warpMatrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    } else {
      // Use warpPerspective for Homography
      cv::warpPerspective(im2, im2_aligned, warpMatrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    }
    cv::putText(im1,"im1",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::putText(im2,"im2",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::putText(im2_aligned,"im2_aligned",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::Mat disIm;
    cv::hconcat(im1,im2_aligned,disIm);
    cv::hconcat(disIm,im2,disIm);
    // Show final result
    if (writeImage) {
      imshow("ImageAlignment", disIm);
      cv::waitKey(1);
      cv::imwrite("image/" + std::to_string(timestamp) + ".png",disIm);
    }
#endif
    return warpMatrix;
  }

  cv::Mat Align(double timestamp,cv::Mat& targetIm,double& err) {
    cv::Mat img2;
    if (PyrDownLevel_ > 1 && cam_ == nullptr) {
      //cv::pyrDown(targetIm,img2,targetIm.size()/PyrDownLevel_);
      cv::resize(targetIm,img2,targetIm.size()/PyrDownLevel_,0,0,CV_INTER_AREA);
    } else {
      if (cam_ != nullptr) {
        img2 = cam_->undistImage(targetIm,mapx_,mapy_);
      } else {
        img2 = targetIm;
      }
    }
    cv::Mat im2 = img2.clone();
    if (targetIm.channels() == 3) {
      cv::cvtColor(targetIm,im2,CV_BGR2GRAY);
    }
    if (Histgram_) {
      cv::equalizeHist(im2,im2);
    }

    cv::Mat warpMatrix;
    if (MotionModel_ > cv::MOTION_HOMOGRAPHY || MotionModel_ < cv::MOTION_TRANSLATION) {
      MotionModel_ = cv::MOTION_HOMOGRAPHY;
    }
    if (MotionModel_ == cv::MOTION_HOMOGRAPHY) {
      warpMatrix = cv::Mat::eye(3, 3, CV_32F);
    } else {
      warpMatrix = cv::Mat::eye(2, 3, CV_32F);
    }
    if (lastImage_.empty()) {
      lastImage_ = im2;
      return warpMatrix;
    }
    cv::Mat im1 = lastImage_.clone();
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, IterationNumber_, TerminationEps_);
    try {
      err = cv::findTransformECC(im1,im2,warpMatrix,MotionModel_,termCriteria);
    } catch (cv::Exception &e) {
      std::cerr << e.msg << std::endl;
      return warpMatrix;
    }
#ifdef PC_VERSION
    cv::Mat im2_aligned;
    if (MotionModel_ != cv::MOTION_HOMOGRAPHY) {
      // Use warpAffine for Translation, Euclidean and Affine
      cv::warpAffine(im2, im2_aligned, warpMatrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    } else {
      // Use warpPerspective for Homography
      cv::warpPerspective(im2, im2_aligned, warpMatrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    }
    cv::putText(im1,"im1",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::putText(im2,"im2",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::putText(im2_aligned,"im2_aligned",cv::Point2f(20,10),cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(0,255,0),1);
    cv::Mat disIm;
    cv::hconcat(im1,im2_aligned,disIm);
    cv::hconcat(disIm,im2,disIm);
    // Show final result
    imshow("ImageAlignment", disIm);
    cv::waitKey(1);
    cv::imwrite("image/" + std::to_string(timestamp) + ".png",disIm);
#endif
    lastImage_ = im2;
    return warpMatrix;
  }

  inline float getFocalLength() const {
    return K_.at<float>(0,0);
  }

  cv::Mat getRectifyIntrinsicMatrix() const {
    return K_;
  }
 private:
  cv::Mat lastImage_;
  bool Histgram_;
  int MotionModel_;
  int IterationNumber_;
  double TerminationEps_;
  int PyrDownLevel_;
  Camera *cam_;
  cv::Mat mapx_,mapy_;
  cv::Mat K_;
};