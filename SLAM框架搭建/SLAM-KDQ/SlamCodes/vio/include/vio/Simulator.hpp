#pragma once
#include "Camera.hpp"
#include "Frame.hpp"
namespace vio{
class Simulator {
 public:
  /** \brief construction 
   * @param cfg  -  config object ptr
   * @param cam  -  camera object ptr
   */ 
  Simulator(const Config* cfg,const Camera* cam);

  /** \breif create 3D land markers and Frames vector using config file
   */ 
  void createLandMarkersAndFrame();

  /** \brief create 3D land markers with config paramters 
   */ 
  void createLandMarkers();

  std::vector<cv::Vec3f> &getLandMarkers() {
    return landMarkers;
  }

 private:
  const Config *cfg_;
  const Camera *cam_;
  std::vector<cv::Vec3f> landMarkers;
  std::vector<FramePtr> framePtrVec;
};
}