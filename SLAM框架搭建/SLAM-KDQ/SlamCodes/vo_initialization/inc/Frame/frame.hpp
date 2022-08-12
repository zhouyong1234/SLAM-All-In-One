#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <map>
class Frame {
  private:
    Eigen::Isometry3d Twc_; 
    bool keyframe_;
    double t_;
    std::map<int,cv::Point2f> keypoints_;
  public:
    Frame();
    Frame(double t,bool keyframe,std::map<int,cv::Point2f>& keypoints);
    ~Frame();

    void setFramePose(Eigen::Isometry3d& Twc) {
      Twc_ = Twc;
    }

    const std::map<int,cv::Point2f>& getKeyPoints() const{
      return keypoints_;
    }
};