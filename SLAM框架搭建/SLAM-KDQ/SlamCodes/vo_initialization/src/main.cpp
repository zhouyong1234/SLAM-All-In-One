#include "imu_motion/imu_motion.hpp"
#include "viz_scene/viz_scene.hpp"
#include "camera_project/camera_project.hpp"
#include "Initialization/Initializator.hpp"

#define VIZ_RANDOM_POINTS_NAME "plane_scene"
#define VIZ_CAMERA_NAME "camera0"

#define PLANE_POINT_FILE "plane_points.csv"
#define IMU_STATE_FILE "imu_state.csv"
#define IMU_NOISE_STATE_FILE "imu_noise_state.csv"
#define IMU_TRUE_POSE_FILE "imu_true_pose.csv"

using namespace viz_scene;
int main(int argc,char** argv) {
  if(argc < 2) {
    cout << "please input config file!" << endl; 
    return -1;
  }
  string configFile = argv[1];
  MotionParam* para = new MotionParam(configFile);
  ImuMotion imuModel(para);
  CameraProject camProjector(para->addPixelNoiseFlg_,para->pixel_noise_,para->cameraConfigFile_,PLANE_POINT_FILE); 
  VizScene planeViz("windows");
  Initializator voInitializator(camProjector.camPtr_,460.0);
  //create plane points and camera object
  planeViz.createRandomPlanePoints(VIZ_RANDOM_POINTS_NAME,Vec3f(0,0,0),Vec3f(0,0,1),50,1,1);
  planeViz.createCameraObject(VIZ_CAMERA_NAME,0.3,Vec2f(0.3,0.4),Vec3f(0,0,0.2),Vec3f(0,0,0.1),Vec3f(0,1.0,0));
  //create record file
  VioDatasInterface::recordCameraPixel(ProjectPointInfo(),PLANE_POINT_FILE,true);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),IMU_STATE_FILE,true);
  VioDatasInterface::recordImuMotionState(ImuMotionState(),IMU_NOISE_STATE_FILE,true);//可以增加噪声
  VioDatasInterface::recordPoseAsTum(ImuMotionState(),IMU_TRUE_POSE_FILE,true);
  //set time
  double t = para->start_t_,last_image_t = para->start_t_;
  double dt = 1.0 / (double)(para->imuFreq_),image_interval_time = 1.0 / (double)(para->imageFreq_);
  cv::Mat planeScene = planeViz.getScenePointCloud(VIZ_RANDOM_POINTS_NAME);

  while(1) {
     if(t  < para->end_t_) {
      //run imu model and get gyro and acc datas 
      ImuMotionState imuState = imuModel.simImuMotion(t);
      //add noise in imu data
      ImuMotionState imuNoiseState = imuModel.addImuNoise(imuState);
      //record infos
      VioDatasInterface::recordImuMotionState(imuState,IMU_STATE_FILE);
      VioDatasInterface::recordImuMotionState(imuNoiseState,IMU_NOISE_STATE_FILE);
      VioDatasInterface::recordPoseAsTum(imuState,IMU_TRUE_POSE_FILE);
      t += dt; 
      //update and display camera pose 
      Eigen::Matrix3d Rwb(imuState.qwi_);
      Eigen::Matrix3d Rwc = Rwb * para->Rbc_;
      Eigen::Vector3d twc = Rwb * para->tbc_ + imuState.pos_;
      Mat3d Rwc_cv;
      eigen2cv(Rwc,Rwc_cv);
      Vec3d twc_cv(twc.x(),twc.y(),twc.z());
      Affine3d Twc_cv(Rwc_cv,twc_cv);
      planeViz.updateCameraPose(VIZ_CAMERA_NAME,Twc_cv);
      //project p3w to image plane
      if (t - last_image_t > image_interval_time) {
        Eigen::Isometry3d Twc;
        Twc.setIdentity();
        Twc.prerotate(Rwc);
        Twc.pretranslate(twc);
        //project and record 
        ProjectPointInfo proPts = camProjector.projectVizPoints(t,planeScene,Twc);
        std::map<int,std::pair<cv::Point3d,cv::Point2i>>::iterator it;
        std::map<int,cv::Point2f> pixelMap;
        for(it = proPts.ptsMap.begin();it != proPts.ptsMap.end();it++){
          pixelMap[it->first] = (cv::Point2f)(it->second.second);
        }
        Frame* kf = new Frame(proPts.t,false,pixelMap);
        Eigen::Matrix3d rotate;
        Eigen::Vector3d trans;
        if(voInitializator.shouldResetReference()) {
          voInitializator.setReferenceFrame(kf);
        } else {
          voInitializator.runInitialization(kf,rotate,trans);
        }
        last_image_t = t;
      }
    } 
    usleep(10);
  }
  printf("Hello VO!\n");
}
