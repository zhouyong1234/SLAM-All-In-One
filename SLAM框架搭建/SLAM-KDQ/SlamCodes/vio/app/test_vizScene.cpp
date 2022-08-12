#include "VizScene.hpp" 
#include "Simulator.hpp"


using namespace vio;
int main(int argc,char **argv) {
 if (argc != 2) {
   printf("Please input config file!\n");
   return -1;
 }
 Config* cfg = new Config(argv[1]);
 Camera* cam = new Camera(cfg);
 Simulator sim(cfg,cam);
 sim.createLandMarkers();
 std::vector<cv::Vec3f>& pts3D = sim.getLandMarkers();
 VizScene vizWindow("test viz");
 vizWindow.createPointClouds("pts",pts3D,cv::viz::Color::green(),4);
 vizWindow.createCameraObject("cam1",0.2,0.2,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(1.0,1.0,0.0),cv::Vec3f(1.0,1.0,0.2),cv::Vec3f(1,1.0,0));
 cv::Affine3d curPose = vizWindow.sceneCamera_["cam1"].cameraPose_;
 float period = CV_PI/100.0;
 float cnt = 0;
 while (!vizWindow.sceneWindowPtr_->wasStopped()) {
   cnt++;
   vizWindow.sceneWindowPtr_->spinOnce(1,false);
   cv::Vec3d t(cos(cnt * period),sin(cnt * period),0);
   cv::Affine3d newPose = curPose.translate(t);
   sim.createLandMarkers();
   std::vector<cv::Vec3f>& pts3D = sim.getLandMarkers();
   vizWindow.updateCameraPose("cam1",newPose);
   vizWindow.updatePointClouds("pts",pts3D);
   vizWindow.showSceneAllCamera();
   vizWindow.showSceneAllPointClouds();
   cv::waitKey(10);
 }
 return 0;
}