//测试结果：radtan模型---发现归一化屏幕的3D点，投影过去的像素点，重新归一化到3D平面，有的点居然和该点坐标不一致，该情况多发生在像素点靠近图像边缘
//        fisheye模型 --- 该模型内参矩阵含义和小孔成像含义不一样，需要归一化平面点先投到球面上在应用内参矩阵得到最终的像素坐标
#include "Camera.hpp"
#include <random>
using namespace vio;
using namespace std;

int main() {

  cv::Mat K = (cv::Mat_<float>(3,3) << 174,   0.0, 160,
                                         0.0, 174, 120,
                                         0.0,   0.0,   1.0);
  cv::Mat D1 = (cv::Mat_<float>(4,1) << -2.917e-02,8.228e-02,5.333e-05,-1.578e-04);
  cv::Mat D2 = (cv::Mat_<float>(4,1) << 0.2742372,-0.3608315,0.1168417,0.0000000);
  bool isFisheye = false;
  Camera radtanCam(K,D1,false,320,240);
  Camera fishCam(K,D2,true,320,240);
  random_device rd;
  mt19937 gen(rd());
  normal_distribution<float> pt3Dx(0,1);//设置均值和标准差
  normal_distribution<float> pt3Dy(0,1);
  vector<cv::Point3f> pts3DVec;
  vector<cv::Point2f> prjuv1,prjuv2;
  //Note: cv::projectPoint need pose from world to camera and rotation use vector not matrix. 
  cv::Mat rcw = (cv::Mat_<float>(3,1) << 0.0,0.0,0.0);
  cv::Mat CtW = (cv::Mat_<float>(3,1) << 0.0,0.0,0.0);
  cv::Mat Rcw;
  cv::Rodrigues(rcw,Rcw);
  std::vector<cv::Mat> rvec,tvec;
  for (size_t i = 0; i < 10; i++) {
    cv::Point3f pt = cv::Point3f(pt3Dx(gen),pt3Dy(gen),1.0);
    cv::Point2f uv1 = radtanCam.project(pt,rcw,CtW);
    cv::Point2f uv2 = fishCam.project(pt,rcw,CtW);
    if (!radtanCam.isInFrame(uv1) || !fishCam.isInFrame(uv2)) {
      i--;
      continue;
    }
    prjuv1.push_back(uv1);
    prjuv2.push_back(uv2);
    pts3DVec.push_back(pt);
  }
  std::cout << "pts3DVec vs normalizedVec vs reject to world" << std::endl;
  for (size_t i = 0; i < 10; i++) {
    cv::Point2f normlizedPt1 = radtanCam.normalized(prjuv1[i]);
    cv::Mat npt1 = (cv::Mat_<float>(3,1) << normlizedPt1.x,normlizedPt1.y,1.0); 
    cv::Mat Wnpt1 = Rcw.t() * (npt1 - CtW);
    cv::Point2f normlizedPt2 = fishCam.normalized(prjuv2[i]);
    cv::Mat npt2 = (cv::Mat_<float>(3,1) << normlizedPt2.x,normlizedPt2.y,1.0); 
    cv::Mat Wnpt2 = Rcw.t() * (npt2 - CtW);
    std::cout << "[radtan]: " << pts3DVec[i] << " vs " << normlizedPt1 << " vs " << Wnpt1.t() << " vs " << prjuv1[i] << std::endl;
    std::cout << "[fisheye]: " << pts3DVec[i] << " vs " << normlizedPt2 << " vs " << Wnpt2.t() << " vs " << prjuv2[i] << std::endl;

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