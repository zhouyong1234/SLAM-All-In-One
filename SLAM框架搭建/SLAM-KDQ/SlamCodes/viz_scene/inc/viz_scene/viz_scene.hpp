#pragma once

#include <iostream>
#include <map>
#include <random>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <opencv2/opencv.hpp>

namespace viz_scene {
using namespace std;
using namespace cv;

class Line {
 public:
  Line() {
    color_ = viz::Color::white();
  }
  Line(Point3d sp, Point3d ep, viz::Color color) :
      startPoint_(sp),
      endPoint_(ep),
      color_(color) {
  }
  Point3d startPoint_;
  Point3d endPoint_;
  viz::Color color_;
};
// Lines Class
class SceneLines {
 public:

  SceneLines() {
    displaySize_ = 1;
    color_ = viz::Color::white();
  }
  SceneLines(int displaySize, viz::Color color) :
      displaySize_(displaySize),
      color_(color) {
    updateFlg_ = false;
  }
  void update(const vector<Line>& lines) {
    lines_.clear();
    lines_ = lines;
    updateFlg_ = true;
  }
  vector<Line> lines_;
  bool updateFlg_;
  cv::viz::Color color_;
  int displaySize_;
};

template<typename T>
class ScenePointCloud {
 public:
  std::vector<T> cloudPoints_;
  bool updateFlg_;
  cv::viz::Color color_;
  int displaySize_;

  ScenePointCloud() {
    color_ = viz::Color::white();
    displaySize_ = 1;
    updateFlg_ = false;
  }

  ScenePointCloud(const std::vector<T> &cloudPts,
                  cv::viz::Color color = cv::viz::Color::white(),
                  bool update = true,
                  int size = 2) {
    cloudPoints_ = cloudPts;
    updateFlg_ = update;
    color_ = color;
    displaySize_ = size;
  }

  void update(const std::vector<T> &pts3D) {
    updateFlg_ = true;
    cloudPoints_.clear();
    cloudPoints_ = pts3D;
  }
};

class CameraObject {
 public:
  CameraObject() {
  };

  CameraObject(string cameraName,
               float coorScalar,
               Vec2f frustumScalar,
               Vec3f pos,
               Vec3f focalPointPos,
               Vec3f y_dirction,
               float recordPathFreq)
      : cameraCoordinateScalar_(coorScalar),
        cameraFrustum_(frustumScalar, 1.0, viz::Color::green()) {
    cameraPose_ = viz::makeCameraPose(pos, focalPointPos, y_dirction);
    cCoorScalarName_ = cameraName + "_coor";
    cFrusName_ = cameraName + "_frustum";
    cPathName_ = cameraName + "_path";
    poseVec_.clear();
    recoreFreq_ = recordPathFreq;
    recordCount_ = 0;
  };
  viz::WCameraPosition cameraCoordinateScalar_;
  viz::WCameraPosition cameraFrustum_;
  vector<Affine3d> poseVec_;
  Affine3d cameraPose_;
  string cCoorScalarName_, cFrusName_, cPathName_;

  void updatePose(const Affine3d &pos) {
    cameraPose_ = pos;
    cameraCoordinateScalar_.setPose(cameraPose_);
    cameraFrustum_.setPose(cameraPose_);
    recordCount_++;
    if (recordCount_ % recoreFreq_ == 0)
      poseVec_.push_back(pos);
  };

  void clearPath() {
    poseVec_.clear();
    recordCount_ = 0;
  }

 private:
  int recoreFreq_;
  int recordCount_;
};

class VizScene {

 public:
  typedef ScenePointCloud<Vec3f> ScenePointCloudType;  //<! cloud maybe has motion or change

  /** \brief construction function
   */
  VizScene(std::string windowName, double scale = 1.0);

  /** \brief disconstruction function
   */
  ~VizScene();

  /** \brief viz window show at spinOnce way
   *
   */
  void windowShowLoopRun();

  /** \brief show pointclouds in parameter sceneCloud_
   */
  void showScenePointClouds();


  /** \brief show pointclouds in parameter sceneCloud_
  */
  void showSceneLines();

  /** \brief create plane filled with random points
   * @param name - name of scene pointcloud set by user
   * @param center_vec - center position of this plane scene in world coordinate
   * @param normal_vec - normal vector of this plane
   * @param nums - number of point cloud
   * @param width - width of plane
   * @param height - height of plane
   * @param thick - if thick !=0,then create cube filled with random points
   */
  bool createRandomPlanePoints(string name, const Vec3f &center_vec, const Vec3f &normal_vec, int nums, float width,
                               float height, float thick = 0);

  /** \brief create pointclouds from 3D point
   * @param ptsName - name of point cloud
   * @param color - display point color
   * @param displaySize - display point size
   */
  bool
  createSceneClouds(std::string ptsName, cv::viz::Color color = cv::viz::Color::white(),
                    int displaySize = 2);

  /** \brief update pointcloud positions which named as ptsname
   * @param ptsName - name of pointcloud
   * @param pts3D - pointcloud
   */
  bool updateSceneClouds(std::string ptsName, const std::vector<cv::Vec3f> &pts3D);

  /** \brief create pointclouds from 3D point
   * @param lineName - name of lines
   * @param color - display point color
   * @param displaySize - display point size
   */
  bool
  createSceneLines(std::string lineName, cv::viz::Color color = cv::viz::Color::white(),
                   int displaySize = 2);

  /** \brief update pointcloud positions which named as ptsname
   * @param lineName - name of pointcloud
   * @param lines - lines 
   */
  bool updateSceneLines(std::string lineName, const std::vector<Line> &lines);

  /** \brief create camera object
   * @param cameraName - camera name
   * @param coorScalar - scalar of camera coordinate
   * @param frustumScalar - frustum Scalar
   * @param pos - camera position
   * @param focalPointPos - focal point position
   * @param y_direction -  y axis direction of camera frame
   */
  bool createCameraObject(string cameraName, float coorScalar, Vec2f frustumScalar, Vec3f pos, Vec3f focalPointPos,
                          Vec3f y_direction);

  /** \brief update pose of camera named by "cameraName"
   * @param cameraName - camera name
   * @param Twc - camera pose
   */
  bool updateCameraPose(const string cameraName, const Affine3d &Twc);

  /** \brief test scene points for increase points z
   *  @param name - scene points name
   */
  void testIncreasePoints(string name);

  /** \brief get point cloud through scene name
   *
   *
   */
  vector<Vec3f> getScenePointCloud(string name);
 private:
  map<string, ScenePointCloudType> sceneCloud_;
  map<string, CameraObject> sceneCamera_;
  map<string, SceneLines> sceneLines_;
  viz::Viz3d *sceneWindowPtr_;
  thread *windowLoopThread_;
  mutex mCloud_;
};
}