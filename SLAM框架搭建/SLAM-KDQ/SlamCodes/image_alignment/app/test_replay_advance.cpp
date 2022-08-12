#include "iostream"
#include "nnstation/BottomImage.hpp"
#include "cmdline.h"
#include "ImageAlignment.hpp"
#include "ConfigLoad.hpp"
#include "map"
#include "mutex"
#include "condition_variable"
#include "eigen3/Eigen/Dense"

cv::Mat TscMatrix(cv::Mat K,cv::Mat Tcb) {
  cv::Mat Rcb = Tcb.colRange(0,3).rowRange(0,3).clone();
  cv::Mat tcb = Tcb.col(3).rowRange(0,3).clone();
  cv::Mat Rsc = K.inv() * Rcb;
  cv::Mat tsc = K.inv() * tcb;
  cv::Mat Tsc = cv::Mat(3,4,CV_32F);
  Rsc.copyTo(Tsc.colRange(0,3).rowRange(0,3));
  tsc.copyTo(Tsc.col(3).rowRange(0,3));
  return Tsc;
}

cv::Mat transformWarp2World(cv::Mat warp,cv::Mat Tsc) {
  cv::Mat warpN;
  if (warp.rows == 3) {
    warpN = warp.clone();
  } else {
    cv::Mat tm = cv::Mat::eye(3,3,CV_32F);
    for (size_t i = 0; i < 2; i++) {
      for (size_t j = 0; j < 3; j++) {
        tm.at<float>(i,j) = warp.at<float>(i,j);
      }
    }
    warpN = tm.clone();
  }
  cv::Mat Tcs;
  cv::invert(Tsc,Tcs,cv::DECOMP_SVD);
  return Tcs * warpN * Tsc;
}

int main(int argc,char** argv) {
  cmdline::parser parser;
  parser.add<std::string>("image_server_url", 'u', "image server url.", false, "tcp://127.0.0.1:11900");
  parser.add<std::string>("camera_config", 'f', "camera config file.", false, "rovio_cheerios.yaml");//建议使用去畸变方式，否则计算的warp可能有零偏
  parser.add<int>("motion_model",'m',"image alignment model",false,1);
  parser.add<int>("iteration_number",'i',"iteration number of image alignment",false,25); //通常迭代十几次可能就收敛了，误差可能在1e-3以内且不会再下降，因此再迭代没有意义
  parser.add<double>("termination_eps",'e',"termination eps for ecc",false, 1e-3);//默认值已经够了，当迭代一定次数后，误差降低很不明显，因此不必要设置大的迭代次数
  parser.add<int>("pyramid_level",'l',"level of pyramid for image alignment",false,4);//金字塔等级越高，相应平移部分越小，L1和L4对比发现，平移部分呈现4倍关系，旋转部分L1波动要小一点
  parser.add<bool>("histogram_enable",'g',"enable equalize histogram flg",false ,false); //当前看对ecc估计效果影响不大
  parser.parse_check(argc,argv);
  std::string imageUrl;
  std::string cameraConfigFile;
  int alignModel = cv::MOTION_EUCLIDEAN;
  int iterationNum = 40;
  double terminationEps = 1e-3;
  int pyramidLevel = 1;
  bool histogramEnable = false;
  imageUrl = parser.get<std::string>("image_server_url");
  cameraConfigFile = parser.get<std::string>("camera_config");
  alignModel = parser.get<int>("motion_model");
  iterationNum = parser.get<int>("iteration_number");
  terminationEps = parser.get<double>("termination_eps");
  pyramidLevel = parser.get<int>("pyramid_level");
  histogramEnable = parser.get<bool>("histogram_enable");
  if (iterationNum <= 0) {
    iterationNum = 10;
  }
  ConfigLoad cfg(cameraConfigFile);
  Camera* cam = nullptr;
  if (cfg.readOK_) {
    cam = new Camera(cfg.K_,cfg.D_,cfg.fisheye_,cfg.width_,cfg.height_);
  }
  ImageAlignment imgAlign(cam,alignModel,iterationNum,terminationEps,pyramidLevel,histogramEnable);
  nnstation::BottomClient bottomClient;
  std::map<double,cv::Mat> imageBuffer;
  std::mutex imgLck;
  std::condition_variable imgCv;
  //cv::Mat Tsc = TscMatrix(imgAlign.getRectifyIntrinsicMatrix(),cfg.Tcb_);
  cv::Mat Tsc = cv::Mat::eye(3,3,CV_32F);
  Tsc.at<float>(0,2) = 0.035;
  std::cout << "Tsc = \n" << Tsc << std::endl;

  bottomClient.connect(imageUrl);
  bottomClient.subscribe([&](const vision::BottomImage &bottomImage) {
    cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                         (char *) bottomImage.image_buffer().c_str()).clone();
    imgLck.lock();
    imageBuffer[bottomImage.timestamp()] = im;
    imgLck.unlock();
    imgCv.notify_all();
  });
  bottomClient.startRecv();
  double last_t = 0;
  std::cout << "WarpMatrix:t,dt,cost,err,a00,a01,t0,a10,a11,t1,w,vx,vy,wvx,wvy" << std::endl;
  std::cout << "WarpW:t,a00,a01,t0,a10,a11,t1" << std::endl;
  while(1) {
    std::unique_lock<std::mutex> lck(imgLck);
    imgCv.wait(lck,[&](){return !imageBuffer.empty();});
    double t = imageBuffer.begin()->first;
    cv::Mat img = imageBuffer.begin()->second.clone();
    imageBuffer.erase(imageBuffer.begin());
    double err;
    const double toc_start  = (double) cv::getTickCount();
    cv::Mat warp1 = imgAlign.Align(t,img,err);
    const double toc_final  = ((double) cv::getTickCount() - toc_start) / cv::getTickFrequency();
    double dt = t - last_t;
    last_t = t;
    if (dt > 10.) {
      continue;
    }
    std::cout << "WarpMatrix:" << t << ","  << dt << "," << toc_final << "," << err << ",";
    for (size_t i = 0; i < warp1.rows; i++) {
      for (size_t j = 0; j < warp1.cols; j++) {
        std::cout << warp1.at<float>(i,j) << ",";
      }
    }
    float tx = warp1.at<float>(0,2) / imgAlign.getFocalLength();
    float ty = warp1.at<float>(1,2) / imgAlign.getFocalLength();
    float vx = tx / dt;
    float vy = ty / dt;
    float theta =  acos(warp1.at<float>(0,0)) ;
    float w = theta / dt;
    Eigen::Vector3f angle(0.,0.,w);
    Eigen::Vector3f tcb(-0.035,-0.04,0);
    Eigen::Vector3f vcross;
    vcross = angle.cross(tcb);
    double wvx = vx + vcross.x();
    double wvy = vy + vcross.y();
    std::cout << theta * 180 / M_PI / dt << "," << vx << ","  << vy << ","<< wvx << "," << wvy << "," << std::endl;
//    for (size_t i = 0; i < 2; i++) {
//      for (size_t j = 0; j < 3; j++) {
//        std::cout << warpW.at<float>(i,j) << ",";
//      }
//    }
//    std::cout << std::endl;
  }
  return 0;
}

