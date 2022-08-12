//测试结果：radtan模型---发现归一化屏幕的3D点，投影过去的像素点，重新归一化到3D平面，有的点居然和该点坐标不一致，该情况多发生在像素点靠近图像边缘
//        fisheye模型 --- 该模型内参矩阵含义和小孔成像含义不一样，需要归一化平面点先投到球面上在应用内参矩阵得到最终的像素坐标
#include "Camera.hpp"
#include <random>
using namespace vio;
using namespace std;

int main() {

  cv::Mat K = (cv::Mat_<double>(3,3) << 174,   0.0, 160,
                                         0.0, 174, 120,
                                         0.0,   0.0,   1.0);
  //cv::Mat D = (cv::Mat_<double>(4,1) << -2.917e-01,8.228e-02,5.333e-05,-1.578e-04);
  cv::Mat D = (cv::Mat_<double>(4,1) << 0.0,0.0,0.0,0.0);
  bool isFisheye = false;
  Camera fishCam(K,D,true,320,240);
  Camera radtanCam(K,D,false,320,240);
  random_device rd;
  mt19937 gen(rd());
  normal_distribution<float> pt3Dx(0,1);//设置均值和标准差
  normal_distribution<float> pt3Dy(0,1);
  vector<cv::Point3f> pts3DVec;
  vector<cv::Point2f> prjuv,uv;
  //Note: cv::projectPoint need pose from world to camera and rotation use vector not matrix. 
  cv::Mat rcw = (cv::Mat_<float>(3,1) << 0.0,0.0,0.0);
  cv::Mat CtW = (cv::Mat_<float>(3,1) << 0.0,0.0,0.0);
  cv::Mat Rcw;
  cv::Rodrigues(rcw,Rcw);
  std::vector<cv::Mat> rvec,tvec;
  for (size_t i = 0; i < 10; i++) {
    cv::Point3f pt = cv::Point3f(pt3Dx(gen),pt3Dy(gen),1.0);
    cv::Point2f uv = radtanCam.project(pt,rcw,CtW);
    prjuv.push_back(uv);
    pts3DVec.push_back(pt);
  }
  radtanCam.project(pts3DVec,uv,rcw,CtW);
  std::cout << "pts3DVec vs normalizedVec vs reject to world" << std::endl;
  for (size_t i = 0; i < 10; i++) {
    cv::Point2f normlizedPt = radtanCam.normalized(uv[i]);
    cv::Mat npt = (cv::Mat_<float>(3,1) << normlizedPt.x,normlizedPt.y,1.0); 
    cv::Mat Wnpt = Rcw.t() * (npt - CtW);
    std::cout << "[3D]: " << pts3DVec[i] << " vs " << normlizedPt << " vs " << Wnpt.t() << std::endl;
  }
  cv::Point2f origin = radtanCam.project(cv::Point3f(0.5, 0.5, 1),rcw,CtW);
  cv::Point2f originFish = fishCam.project(cv::Point3f(0.5, 0.5, 1),rcw,CtW);
  std::cout << "origin = " << origin << std::endl; 
  cv::Point2f normalizedOrigin = radtanCam.normalized(origin);
  cv::Point2f normalizedOrigin2 = fishCam.normalized(originFish);
  std::cout << "[Pixel]:RadTan = " << origin << " vs Fisheye = " << originFish << std::endl;
  std::cout << "[Normalized]:RadTan = " << normalizedOrigin << " vs Fisheye = " << normalizedOrigin2 << std::endl;
  std::cout << "Test Result:1）fisheye模型需要内参数矩阵是指先把归一化平面上的点转到球面上之后再应用的,所以即便和radtan内参数一样,其最终像素点位置也是不一样的(即便畸变全设置为0);\n" 
            << "2）radtan模型在使用的时候，有的点归一化不能回到正确的位置上，该问题需要确认原因"  << std::endl;
  return 0;
}