#include "VizScene.hpp" 
#include "Simulator.hpp"
#include "PnpSolver.hpp"

using namespace vio;

#define PI 3.1415926

int main(int argc,char **argv) {
 if (argc != 2) {
   printf("Please input config file!\n");
   return -1;
 }
 Config* cfg = new Config(argv[1]);
 Camera* cam = new Camera(cfg);
 PnpSolver solver(cfg);
 Simulator sim(cfg,cam);
 sim.createLandMarkers();
 std::vector<cv::Vec3f>& pts3D = sim.getLandMarkers();
 VizScene vizWindow("test viz");
 vizWindow.createPointClouds("pts",pts3D,cv::viz::Color::green(),4);
 vizWindow.createCameraObject("cam1",0.2,0.2,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(1.0,1.0,1.0),cv::Vec3f(1.0,1.0,-1.),cv::Vec3f(0,1.0,0));
 vizWindow.createCameraObject("cam2",0.2,0.2,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(1.0,1.0,1.0),cv::Vec3f(1.0,1.0,-1.),cv::Vec3f(0,1.0,0));
 cv::Affine3d curPose = vizWindow.sceneCamera_["cam1"].cameraPose_;
 cv::Affine3d solvedPose = curPose;
 float period = CV_PI/100.0;
 float cnt = 0;
 while (!vizWindow.sceneWindowPtr_->wasStopped()) {
   cnt++;
   vizWindow.sceneWindowPtr_->spinOnce(1,false);
   cv::Vec3d t(cos(cnt * period),sin(cnt * period),0);
   cv::Affine3d newPose = curPose.translate(t);
   std::vector<cv::Point2f> pts2D;
   cv::Affine3d::Mat4 poseMat = newPose.inv().matrix;
   cv::Mat CrW = (cv::Mat_<float>(3,3) << poseMat(0,0),poseMat(0,1),poseMat(0,2),
                                           poseMat(1,0),poseMat(1,1),poseMat(1,2),
                                           poseMat(2,0),poseMat(2,1),poseMat(2,2));
   cv::Mat ctw = (cv::Mat_<float>(3,1) << poseMat(0,3),poseMat(1,3),poseMat(2,3));
   cv::Mat crw;
   cv::Rodrigues(CrW,crw);
   std::vector<cv::Point3f> pts3d;
   std::vector<cv::Point2f> uv;
   cv::Point3f camPose(newPose.matrix(0,3),newPose.matrix(1,3),newPose.matrix(2,3));
   for (auto pt3D:pts3D) {
     cv::Point3f pt3DP(pt3D[0],pt3D[1],pt3D[2]);
     cv::Point2f pixel = cam->project(pt3DP,crw,ctw);
     if (cam->isInFrame(pixel)) {
       cv::Point3f ptVec = camPose - pt3DP;
       float depth = sqrt(ptVec.dot(ptVec));
       cv::Point2f normalizedPt = cam->normalized(pixel);
       cv::Point3f norm3DPt(normalizedPt.x,normalizedPt.y,1);
       cv::Point3f pt3DInCam = depth * norm3DPt / sqrt(norm3DPt.dot(norm3DPt));
       cv::Point3f pt3DInWorld = newPose * pt3DInCam;
       uv.push_back(normalizedPt);
       pts3d.push_back(pt3DP);
       //std::cout << pt3DP << " vs " << pt3DInWorld << std::endl;
     } 
   }
   std::cout << "match size = " << uv.size() << std::endl;
   if (uv.size() > 5) {
     cv::Mat r,t;
     std::vector<int> inlier;
     solver.solveByPnp(uv,pts3d,cam->fx(),r,t,inlier);
     solvedPose.rotation(r);
     cv::Mat tt = t + (cv::Mat_<double>(3,1) << 0,0,0.2);
     solvedPose.translation(tt);
     vizWindow.updateCameraPose("cam2",solvedPose.inv()); 
     std::cout << "r = " << r  << "\n" << " t = " << t << std::endl; 
   }
   vizWindow.updateCameraPose("cam1",newPose);
   vizWindow.showSceneAllCamera();
   vizWindow.showSceneAllPointClouds();
   cv::waitKey(30);
 }
 return 0;
}