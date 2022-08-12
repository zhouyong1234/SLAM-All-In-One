#include "VizScene.hpp" 
#include "Simulator.hpp"
#include "Estimator.hpp"
#include "d435i.hpp"

using namespace vio;


int main(int argc,char **argv) {
 if (argc != 2) {
   printf("Please input config file!\n");
   return -1;
 }
 Config* cfg = new Config(argv[1]);
 CameraPtr cam(new Camera(cfg));
 Estimator estimator(argv[1]);
 D435I d435i;
 d435i.start();

 std::vector<cv::Vec3f> pts3D;
 pts3D.emplace_back(0,0,0);
 pts3D.emplace_back(0,0,0);
 VizScene vizWindow("test viz",0.05);
 vizWindow.createCameraObject("d435i",0.05,0.05,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(0.0,0.0,0.0),cv::Vec3f(0.0,0.0,1.0),cv::Vec3f(0,1.0,0));
 vizWindow.createPointClouds("cameraPts",pts3D,cv::viz::Color::red(),4);

 while (!vizWindow.sceneWindowPtr_->wasStopped()) {
   vizWindow.sceneWindowPtr_->spinOnce(1,false);
   double timestamp;
   cv::Mat leftImg,rightImg;
   if (d435i.getInfraredImages(timestamp,leftImg,rightImg)) {
    Frame *frame = new Frame(timestamp,leftImg,cam);
    estimator.update(FramePtr(frame),true);
    std::cout << "state = " << estimator.getEstimatorState() << std::endl;
    cv::Mat Rwc,WtC;
    if (estimator.getCurrentPose(Rwc,WtC) && estimator.getEstimatorState() == 2) {
      cv::Affine3d Twc(Rwc,WtC);
      vizWindow.updateCameraPose("d435i",Twc);
      vizWindow.updatePointClouds("cameraPts",estimator.getFeatsInWorld());
    } else {
      vizWindow.clearCameraPath("d435i");
    }
   } 
   vizWindow.showSceneAllCamera();
   vizWindow.showSceneAllPointClouds();
   //cv::waitKey(30);
 }
 return 0;
}