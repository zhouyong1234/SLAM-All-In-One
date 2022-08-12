#include <iostream>
#include "DepthFilter.hpp"
#include "viz_scene/viz_scene.hpp"
#include "imu_motion/imu_motion.hpp"
#include "utilities/access_file.hpp"
#include "camera_project/camera_project.hpp"
#include "CvCamera.hpp"
using namespace std;
using namespace viz_scene;
using namespace depth_filter;

int main(int argc,char** argv) {
  if(argc < 2) {
    cout << "please input config file!" << endl; 
    return -1;
  }
  //Set simulation model
  string configFile = argv[1];
  MotionParam* para = new MotionParam(configFile);
  ImuMotion imuModel(para);
  CameraProject camProjector(para->addPixelNoiseFlg_,para->pixel_noise_,para->cameraConfigFile_,"log/points_test.csv");
  //Create simulation environment
  VizScene vizScene("test");
  vizScene.createSceneClouds("uv_camera",viz::Color::white(),2);
  vizScene.createSceneClouds("xyz_world",viz::Color::red(),4);
  vizScene.createSceneLines("p_sigma",viz::Color::blue(),1);
  vizScene.createRandomPlanePoints("test plane",Vec3f(0,0,0),Vec3f(0,0,1),100,10,10);
  vizScene.createCameraObject("test camera",0.3,Vec2f(0.3,0.4),Vec3f(0,0,0.2),Vec3f(0,0,0.1),Vec3f(0,1.0,0));
  vector<Vec3f> test_plane = vizScene.getScenePointCloud("test plane");
  //Record files you should mkdir log folder
  VioDatasInterface::recordCameraPixel(ProjectPointInfo(),"log/points_test.csv",true);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),"log/test.csv",true);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),"log/noise_test.csv",true);
  VioDatasInterface::recordPoseAsTum(ImuMotionState(),"log/imu_pose.csv",true);
  VioDatasInterface::recordPoseAsTum(ImuMotionState(),"log/real_pose.csv",true);
  //Construct depth filter
  CvCamera cam(para->cameraConfigFile_);
  Eigen::Matrix3d K;
  cv::cv2eigen(cam.K(),K);
  DepthFilter dF(K,cam.width(),cam.height(),[&](const std::vector<SeedPoint>& clouds){
    vector<Vec3f> seedPoints;
    vector<Line> seedLines;
    for (int i = 0; i < clouds.size(); ++i) {
      Vec3f mean(clouds[i].meanPtsInWorld.x(),clouds[i].meanPtsInWorld.y(),clouds[i].meanPtsInWorld.z());
      seedPoints.push_back(mean);
      Point3d min(clouds[i].minPtsInWorld.x(),clouds[i].minPtsInWorld.y(),clouds[i].minPtsInWorld.z());
      Point3d max(clouds[i].maxPtsInWorld.x(),clouds[i].maxPtsInWorld.y(),clouds[i].maxPtsInWorld.z());
      seedLines.emplace_back(min,max,cv::viz::Color::green());
    }
    vizScene.updateSceneLines("p_sigma",seedLines);
    vizScene.updateSceneClouds("xyz_world",seedPoints);
  });
  // set timestamp sequence
  double t = para->start_t_,last_image_t = para->start_t_;
  double dt = 1.0 / (double)(para->imuFreq_),image_interval_time = 1.0 / (double)(para->imageFreq_);
  cv::Mat rawImage(cam.height(),cam.width(),CV_8UC3, Scalar(255,255,255));
  cv::Mat img = rawImage.clone();
  // set position noise
  std::mt19937 gen{12345};
  std::uniform_real_distribution<float> titlePos(-para->simPositionNoiseXY_,para->simPositionNoiseXY_);
  std::uniform_real_distribution<float> verticalPos(-para->simPositionNoiseZ_,para->simPositionNoiseZ_);
  Eigen::Matrix3d transNoise;
  transNoise << para->simPositionNoiseXY_ * para->simPositionNoiseXY_,0,0,
                0,para->simPositionNoiseXY_ * para->simPositionNoiseXY_,0,
                0,0,para->simPositionNoiseZ_ * para->simPositionNoiseZ_;
  // set control flg
  bool pauseFlg = false;
  int nextFlg = 0;
  while(1) {
    //vizScene.testIncreasePoints("test plane");
    bool imgUpdateFlg = false;
    if(t  < para->end_t_) {
      ImuMotionState imuState = imuModel.simImuMotion(t);
      ImuMotionState imuNoiseState = imuModel.addImuNoise(imuState);
      VioDatasInterface::recordImuMotionState(imuState,"log/test.csv");
      VioDatasInterface::recordPoseAsTum(imuState,"log/real_pose.csv");
      VioDatasInterface::recordImuMotionState(imuNoiseState,"log/noise_test.csv");
      t += dt; 
      Eigen::Matrix3d Rwb(imuState.qwi_);
      Eigen::Matrix3d Rwc = Rwb * para->Rbc_;
      Eigen::Vector3d twc = Rwb * para->tbc_ + imuState.pos_;
      Mat3d Rwc_cv;
      eigen2cv(Rwc,Rwc_cv);
      Vec3d twc_cv(twc.x(),twc.y(),twc.z());
      Affine3d Twc_cv(Rwc_cv,twc_cv);
      if (t - last_image_t > image_interval_time) {
        imgUpdateFlg = true;
        Eigen::Isometry3d Twc;
        Twc.setIdentity();
        Twc.prerotate(Rwc);
        Twc.pretranslate(twc);
        ProjectPointInfo corners = camProjector.projectVizPoints(t,test_plane,Twc);
        DepthFilter::FramePtr fptr;
        fptr.reset(new Frame(corners.Twc.inverse(),Eigen::Matrix3d::Zero(),1.));
        vector<Vec3f> camPoints,camShowPoints;
        img = rawImage.clone();
        for(auto it = corners.ptsMap.begin(); it != corners.ptsMap.end(); it++) {
          int id = it->first;
          Eigen::Vector2d uv(it->second.second.x,it->second.second.y);
          Eigen::Vector3d f;
          camProjector.camPtr_->liftSphere(uv,f);
          Eigen::Vector3d fw = Twc * f;
          camPoints.emplace_back(f.x(),f.y(),f.z());
          camShowPoints.emplace_back(fw.x(),fw.y(),fw.z());
          Corner c(id,1.,f);
          fptr->insertCorner(c);
#ifdef  VERBOOSE
          cv::circle(img,it->second.second,3,cv::Scalar(255,0,0));
          cv::putText(img, to_string(id),it->second.second,1,1,cv::Scalar(0,0,255));
#endif
        }
        vizScene.updateSceneClouds("uv_camera",camShowPoints);
        dF.addFrame(fptr);
        dF.addKeyframe(fptr,5.,5);
        last_image_t = t;
      }
      vizScene.updateCameraPose("test camera",Twc_cv);
    } else {
      std::vector<ImuMotionState> imuData;
      VioDatasInterface::readImuMotionState(imuData,"log/test.csv");
      std::cout << "imu data size = " << imuData.size() << std::endl;
      (imuData.end()-1)->printData();
      imuModel.testImuMotionData("log/test.csv","log/pose.csv");
      imuModel.testImuMotionData("log/noise_test.csv","log/noise_pose.csv");
      std::vector<ProjectPointInfo> proPtsVec;
      VioDatasInterface::readCameraPixel("log/points_test.csv",proPtsVec);
      for (size_t i = 0; i < proPtsVec.size(); i++) {
        ProjectPointInfo ptsInfo = proPtsVec[i];
        std::cout << ptsInfo.t << "\n" << ptsInfo.Twc.matrix() << std::endl;
        for (auto &p:ptsInfo.ptsMap) {
          Eigen::Vector3d p3W(p.second.first.x, p.second.first.y, p.second.first.z);
          Eigen::Vector2d p2D(p.second.second.x, p.second.second.y);
          double scale = (ptsInfo.Twc.translation() - p3W).norm();
          Eigen::Vector3d p3W_map = camProjector.pixelInverseProjectToWorld(ptsInfo.Twc, p2D, scale);
          std::cout << p.first << " " << p.second.first << " " << p.second.second << " " << p3W_map.transpose()
                    << std::endl;
        }
      }
      break;
    }
#ifdef VERBOOSE
    while(1) {
      cv::imshow("corners",img);
      char key = cv::waitKey(30);
      if (key == ' ') {
        pauseFlg = !pauseFlg;
        nextFlg = 0;
      } else if (key == 'n') {
        nextFlg++;
      }
      switch (nextFlg) {
        case 0:
          break;
        case 1:
          pauseFlg = true;
          break;
        case 2:
          pauseFlg = false;
          if (imgUpdateFlg)
            nextFlg = 1;
          break;
        default:
          break;
      }

      if (pauseFlg == false) {
        break;
      }
    }
#else
    usleep(dt * 1e6);
#endif
  }
  return 0;
} 
