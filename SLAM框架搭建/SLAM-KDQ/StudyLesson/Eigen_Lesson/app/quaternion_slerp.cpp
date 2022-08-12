#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "eigen3/Eigen/Dense"
using namespace cv::viz;

int main() {
  Viz3d scene1("slerp quaternion");
  scene1.showWidget("widget coordinate",cv::viz::WCoordinateSystem(0.3));
  
  Eigen::Vector3d point1(0,1,1);  
  Eigen::AngleAxisd axis0(M_PI,Eigen::Vector3d(0,1,0));
  Eigen::AngleAxisd axis1(2 * M_PI,Eigen::Vector3d(0,0,1));

  Eigen::Quaterniond q0(axis0);
  Eigen::Quaterniond q1(axis1);
  Eigen::Vector3d p0 = q0 * point1;
  Eigen::Vector3d p1 = q1 * point1;

  WLine l0(cv::Point3f(0,0,0),cv::Point3f(p0.x(),p0.y(),p0.z()),Color::red());
  WLine l1(cv::Point3f(0,0,0),cv::Point3f(p1.x(),p1.y(),p1.z()),Color::green());
  l0.setRenderingProperty(LINE_WIDTH,4);
  l1.setRenderingProperty(LINE_WIDTH,4);
  scene1.showWidget("l0",l0);
  scene1.showWidget("l1",l1);

  double t = 0;
  while (!scene1.wasStopped())
  {
    scene1.spinOnce(1,false);
    t += 0.05;
    Eigen::Quaterniond q_t = q0.slerp(t,q1);
    Eigen::Vector3d p_t = q_t * point1;
    WLine l_t(cv::Point3f(0,0,0),cv::Point3f(p_t.x(),p_t.y(),p_t.z()),Color::bluberry());
    scene1.showWidget("l_t",l_t);
    cv::waitKey(30);
  }
  
 
  std::cout << "Hello slerp" << std::endl;
  return 0;

}