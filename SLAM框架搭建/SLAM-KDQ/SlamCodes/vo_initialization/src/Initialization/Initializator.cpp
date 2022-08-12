#include <numeric>
#include "Initialization/Initializator.hpp"
#include <opencv2/core/eigen.hpp>

Initializator::Initializator(CameraPtr camPtr,float focalLength)
:camPtr_(camPtr),focalLength_(focalLength),resetReferFlg_(true) {
  ref_ = nullptr;
  std::cout << "Focal Length = " << focalLength_ << std::endl;
}

Initializator::~Initializator() {

}

bool Initializator::setReferenceFrame(Frame* ref) {
  if(ref_ != nullptr) {
    delete ref_;
    ref_ = nullptr;
  }
  if(ref->getKeyPoints().size() > 25) {
    ref_ = ref;
    resetReferFlg_ = false;
    return true;
  } else {
    std::cout << "keypoints size = " << ref->getKeyPoints().size() << std::endl; 
  }
  return false;
}

void Initializator::matchPoints(const Frame* ref,const Frame* cur) {
  auto refPts = ref->getKeyPoints();
  auto curPts = cur->getKeyPoints();
  if( refPts.empty() || curPts.empty() ) {
    return;
  }
  std::map<int,cv::Point2f>::iterator it;
  for(it = curPts.begin();it != curPts.end();it++) {
    if(refPts.count(it->first)) {
      refKeyPoints_.push_back(undistedPts(refPts[it->first]));
      curKeyPoints_.push_back(undistedPts(it->second));
    }
  }
}

cv::Point2f Initializator::undistedPts(const cv::Point2f& distPoint) {
  Eigen::Vector2d uv(distPoint.x,distPoint.y);
  Eigen::Vector3d undistPt;
  camPtr_->liftProjective(uv,undistPt);
  return cv::Point2f(undistPt.x()/undistPt.z(),undistPt.y()/undistPt.z());
}

bool Initializator::runInitialization(const Frame* cur,Eigen::Matrix3d& rotate,Eigen::Vector3d& trans) {
  refKeyPoints_.clear();
  curKeyPoints_.clear();
  if(ref_ == nullptr) {
    std::cout << "no reference frame" << std::endl;
    return false;
  }
  matchPoints(ref_,cur);
  if(curKeyPoints_.size() < 20) {
    std::cout << "matched point size " << curKeyPoints_.size() << " is too small!" << std::endl;
    resetReferFlg_ = true;
    return false;
  }

  float parallexAver,parallexVar;
  checkParallex(parallexAver,parallexVar);
  if(parallexAver * focalLength_ < 10.0) {
    std::cout << "average parallex " << parallexAver * focalLength_ <<  " is too small" << std::endl;
    return false; 
  }

  cv::Mat mask;
  cv::Mat F = cv::findFundamentalMat(refKeyPoints_,curKeyPoints_,cv::FM_RANSAC,3.0/focalLength_,0.99,mask);
  cv::Mat R,t;
  cv::Mat K = (cv::Mat_<double>(3,3)  << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);
  int inlierSize = cv::recoverPose(F,refKeyPoints_,curKeyPoints_,K,R,t,mask);
  std::cout << "mask-F = " << mask << std::endl;
  if(inlierSize > 15) {
    cv::cv2eigen(R,rotate);
    cv::cv2eigen(t,trans);
    std::cout << "F : \n" << "R = \n" << rotate  << std::endl << "t = " << trans.transpose() << std::endl; 
  }

  cv::Mat mask2;
  cv::Mat H = cv::findHomography(refKeyPoints_,curKeyPoints_,cv::RANSAC,3.0/focalLength_,mask2);
  std::cout << "mask-H = " << mask2 << std::endl;
  std::vector<cv::Mat> R2,t2,normal;
  int inlierSize2 = cv::decomposeHomographyMat(H,K,R2,t2,normal);
  //int inlierSize2 = 0;
  /*
  for (size_t i = 0; i < mask2.size(); i++) {
    if (mask2[i]) {
      inlierSize2++;
    }
  }*/

  if(inlierSize2 < 15 && inlierSize2 < 15) {
    return false;
  }
  if(inlierSize2 > inlierSize) {
    //rotate = rorate2;
    //trans = trans2;
  }
  resetReferFlg_ = false;
  return true;
}

float Initializator::checkFundmentalMatrix(cv::Mat F) {
  
}



void Initializator::checkParallex(float& parallexAver,float& parallexVar) {
  if (refKeyPoints_.empty() 
      || curKeyPoints_.empty() 
      || refKeyPoints_.size() != curKeyPoints_.size()) {
    std::cout << "ref and cur frame points not matched!" << std::endl;
    return;
  }
  float parallexSum = 0;
  std::vector<float> parallexVec;
  for( int i = 0;i < curKeyPoints_.size();i++ ) {
    Eigen::Vector2f refPoint(refKeyPoints_[i].x,refKeyPoints_[i].y);
    Eigen::Vector2f curPoint(curKeyPoints_[i].x,curKeyPoints_[i].y);
    parallexVec.push_back((refPoint - curPoint).norm());
  } 
  
	parallexAver = std::accumulate(std::begin(parallexVec), std::end(parallexVec), 0.0);
  parallexAver /=  parallexVec.size(); 
  float accum = 0;
	std::for_each (std::begin(parallexVec), std::end(parallexVec), [&](const float d) {
		accum  += (d-parallexAver)*(d-parallexAver);
	});
	parallexVar = sqrt(accum/(parallexVec.size()-1));
}
