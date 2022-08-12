#include "viz_scene/viz_scene.hpp"
#include "imu_motion/imu_motion.hpp"
#include "utilities/access_file.hpp"
#include "camera_project/camera_project.hpp"

using namespace viz_scene;
int main(int argc,char** argv){
  if(argc < 2) {
    cout << "please input config file!" << endl; 
    return -1;
  }
  string configFile = argv[1];
  MotionParam* para = new MotionParam(configFile);
  ImuMotion imuModel(para);
  CameraProject camProjector(para->addPixelNoiseFlg_,para->pixel_noise_,para->cameraConfigFile_,"log/points_test.csv");
  VizScene vizScene("test");
  VioDatasInterface::recordCameraPixel(ProjectPointInfo(),"log/points_test.csv",true);
  vizScene.createRandomPlanePoints("test plane",Vec3f(0,0,0),Vec3f(0,0,1),100,4,4);
  vector<Vec3f> test_plane = vizScene.getScenePointCloud("test plane");
  vizScene.createCameraObject("test camera",0.3,Vec2f(0.3,0.4),Vec3f(0,0,0.2),Vec3f(0,0,0.1),Vec3f(0,1.0,0));
  double t = para->start_t_,last_image_t = para->start_t_;
  double dt = 1.0 / (double)(para->imuFreq_),image_interval_time = 1.0 / (double)(para->imageFreq_);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),"log/test.csv",true);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),"log/noise_test.csv",true);
  VioDatasInterface::recordPoseAsTum(ImuMotionState(),"log/imu_pose.csv",true);
  VioDatasInterface::recordPoseAsTum(ImuMotionState(),"log/real_pose.csv",true);

  while(1) {
    //vizScene.testIncreasePoints("test plane");
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
        Eigen::Isometry3d Twc;
        Twc.setIdentity();
        Twc.prerotate(Rwc);
        Twc.pretranslate(twc);
        camProjector.projectVizPoints(t,test_plane,Twc);
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
      for (size_t i = 0; i < proPtsVec.size(); i++)
      {
        ProjectPointInfo ptsInfo = proPtsVec[i];
        std::cout << ptsInfo.t << "\n" << ptsInfo.Twc.matrix() << std::endl ;
        for(auto &p:ptsInfo.ptsMap) {
          Eigen::Vector3d p3W(p.second.first.x,p.second.first.y,p.second.first.z);
          Eigen::Vector2d p2D(p.second.second.x,p.second.second.y);
          double scale = (ptsInfo.Twc.translation() - p3W).norm();
          Eigen::Vector3d p3W_map = camProjector.pixelInverseProjectToWorld( ptsInfo.Twc,p2D,scale);
          std::cout << p.first << " " << p.second.first << " " << p.second.second << " " << p3W_map.transpose() << std::endl;
        }
      }
      
      break;
    }
    usleep(100);
  }
  return 0;
}