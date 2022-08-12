#pragma once
#include <fstream>
#include "Config.hpp"
#include "FeatureManager.hpp"
namespace vio{

class Initializator {
 public:

  /** \brief construction function
   * @param cfg  ---  config object ptr
   * @param cam  ---  camera object ptr 
   */ 
  Initializator(const Config* cfg,Camera* cam);

  /** \brief initialization of slam include pose and features
   * @param refFrame   ---   reference frame ptr
   * @param curFrame   ---   current frame ptr
   * @param fm         ---   feature manager object
   * @return return initialization success flag
   */ 
  bool initPoseAndMap(FramePtr refFrame,FramePtr curFrame,FeatureManager& fm);

  /** \brief Calculate pose of two frames with overlap features through decomposing homography matrix
   *  @param refFrame  ---  referance frame ptr
   *  @param curFrame  ---  current frame ptr
   *  @param pts3D     ---  triangulating points 
   *  @return  success flag of calculating pose  
   */ 
  bool initializeFromHomography(FramePtr  refFrame,FramePtr  curFrame,std::map<uint64_t,cv::Point3f>& pts3D);

  /** \brief Calculate homography matrix of two frames with overlap features  
   *  @param H            --- homegraphy matrix
   *  @param inliers      --- inlier flg array that reprojected error less than @ReprojectErrThr_
   *  @param focalLength  --- focal length 
   *  @param refNormFeats --- normalized points in referance frame
   *  @param curNormFeats --- normalized points in curent frame
   *  @return  success flag of calculating homography
   */ 
  bool calHomography(cv::Mat &H,
                     std::vector<uchar> &inliers,
                     float focalLength,
                     const std::vector<cv::Point2f> &refNormFeats,
                     const std::vector<cv::Point2f> &curNormFeats);

  /** \brief select best pose from pose vector through check all features 3d point is or not in the front of cameras
   *  @param ptsInWorld   --- 3D points through triangulation
   *  @param R            --- Rotation Matrix Array from reference to current frame
   *  @param t            --- translate Matrix Array from reference to current frame
   *  @param n            --- normal Vector Array
   *  @param inliers      --- inliers at @calHomography
   *  @param idVec        --- vector of features' index
   *  @param refNormFeats --- normalized features in reference frame
   *  @param curNormFeats --- normalized features matched with reference frame in the current frame
   */ 
  int checkRt(std::vector< std::map<uint64_t,cv::Point3f> > &ptsInWorld,
                        const std::vector<cv::Mat> &R,
                        const std::vector<cv::Mat> &t,
                        const std::vector<cv::Mat> &n,
                        const std::vector<uchar> &inliers,
                        const std::vector<uint64_t> &idVec,
                        const std::vector<cv::Point2f> &refNormFeats,
                        const std::vector<cv::Point2f> &curNormFeats);
  /** \brief Check homography properity 
   *  @param H --- homography matrix
   *  @return homography is ok or not
   */ 
  inline bool checkHomography(const cv::Mat &H); 

  /** \brief get 3D coordinate through triangulating feature points 
   * @param kp1  --- pixel coordinate in the first frame 
   * @param kp2  --- pixel coordinate in the second frame
   * @param P1   --- pose of first frame in the world 
   * @param P2   --- pose of second frame in the world
   * @param x3D  --- point coordinate in the world
   */ 
  bool triangulate(const cv::Point2f &kp1,
                             const cv::Point2f &kp2,    
                             const cv::Mat &P1,         
                             const cv::Mat &P2,          
                             cv::Point3f &pts3d);

  /** \brief check matched corners middle disparitie,make sure this value not too small
   * @param refCorners  --- corner vector in the reference frame
   * @param curCorners  --- corner vector in the current frame 
   */ 
  bool checkCornerDisparities(std::vector<cv::Point2f>& refCorners,
                         std::vector<cv::Point2f>& curCorners);
 private:
  std::string moduleName_;
  Camera *camera_;
  const float MinDisparity;
  const int InitialMinMatchedPointNum;
  const float InitialReprojectErr;
  const float HomographyTransformErr;
};
}

