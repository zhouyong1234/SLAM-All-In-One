#include <random>
#include "VizScene.hpp"
#include "Simulator.hpp"
#include "Estimator.hpp"

using namespace vio;

#define PI 3.1415926

int main(int argc,char **argv) {
  if (argc != 2) {
    printf("Please input config file!\n");
    return -1;
  }
  Config* cfg = new Config(argv[1]);
  Estimator estimator(argv[1]);
  const CameraPtr cam = estimator.getCameraPtr();
  Simulator sim(cfg,cam.get());
  sim.createLandMarkers();
  std::vector<cv::Vec3f>& pts3D = sim.getLandMarkers();
  VizScene vizWindow("test viz");
  vizWindow.createPointClouds("pts",pts3D,cv::viz::Color::green(),4);
  vizWindow.createPointClouds("cameraPts",pts3D,cv::viz::Color::red(),4);
  vizWindow.createCameraObject("cam1",0.2,0.2,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(0.0,0.0,0.0),cv::Vec3f(0.0,0.0,1.0),cv::Vec3f(0,1.0,0));
  vizWindow.createCameraObject("cam2",0.2,0.2,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(0.0,0.0,0.0),cv::Vec3f(0.0,0.0,1.0),cv::Vec3f(0,1.0,0));
  cv::Affine3d curPose = vizWindow.sceneCamera_["cam1"].cameraPose_;
  cv::Affine3d solvedPose = curPose;
  float period = 0.033;
  float cnt = 0;
  double timestamp = 0.;
  float noiseAmp = 1.0;
  std::random_device sd;
  std::mt19937_64 genorator(sd());
  std::uniform_real_distribution<float> noise(-1,1);
  while (!vizWindow.sceneWindowPtr_->wasStopped()) {
    cnt++;
    timestamp += cnt * period;
    vizWindow.sceneWindowPtr_->spinOnce(1,false);
    cv::Vec3d t(3. * cos(cnt * period),3.0 * sin(cnt * period),0 * (cos(cnt * period) + sin(cnt * period)));
    cv::Affine3d newPose = curPose.translate(t);
    std::vector<cv::Point2f> pts2D;
    cv::Affine3d::Mat4 poseMat = newPose.inv().matrix;
    cv::Mat CrW = (cv::Mat_<float>(3,3) << poseMat(0,0),poseMat(0,1),poseMat(0,2),
      poseMat(1,0),poseMat(1,1),poseMat(1,2),
      poseMat(2,0),poseMat(2,1),poseMat(2,2));
    cv::Mat ctw = (cv::Mat_<float>(3,1) << poseMat(0,3),poseMat(1,3),poseMat(2,3));
    cv::Mat crw;
    cv::Rodrigues(CrW,crw);
    std::map<uint64_t,cv::Point2f> feats;
    cv::Point3f camPose(newPose.matrix(0,3),newPose.matrix(1,3),newPose.matrix(2,3));
    uint64_t id = 0;

    for (auto pt3D:pts3D) {
      id++;
      cv::Point3f pt3DP(pt3D[0],pt3D[1],pt3D[2]);
      cv::Point2f pixel = cam->project(pt3DP,crw,ctw);
      cv::Point2f noisePixel = pixel + cv::Point2f(noiseAmp * noise(genorator),noiseAmp * noise(genorator));
      if (cam->isInFrame(noisePixel)) {
        feats[id] = noisePixel;
      }
    }
    Frame *frame = new Frame(timestamp,feats,cam);
    estimator.update(FramePtr(frame),false);
    cv::Mat Rwc,WtC;
    if (estimator.getCurrentPose(Rwc,WtC) && estimator.getEstimatorState() == 2) {
      cv::Affine3d Twc(Rwc,WtC);
      vizWindow.updateCameraPose("cam2",Twc);
      vizWindow.updatePointClouds("cameraPts",estimator.getFeatsInWorld());
    } else {
      vizWindow.clearCameraPath("cam2");
    }
    vizWindow.updateCameraPose("cam1",newPose);
    vizWindow.showSceneAllCamera();
    vizWindow.showSceneAllPointClouds();
    cv::waitKey(30);
  }
  return 0;
}