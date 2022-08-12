#pragma once
#include <iostream>
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "Frame.hpp"
#include "FeatureManager.hpp"
#include "Camera.hpp"
#include <opencv2/core/eigen.hpp>

using namespace vio;
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
G2O_USE_OPTIMIZATION_LIBRARY(csparse); //csparse运行速度更快
class BAG2O {
 public:
  /** \brief construct ba solver by setting various type of solver
   *
   * @param solverName --- set optimization algrithm、vertex freedom and linear solver type
   *                   --- lm_fix6_3_csparse : lm - L-M optimization algrithm, fix6_3 - binary edge include 6-freedom vertex and 3-freedom vertex, csparse - linear solver
   * @param robustCoreName --- robust core function type,etc huber,cuchy ...
   * @param structureOnly  --- no ideal
   */
  BAG2O(std::string solverName = "lm_fix6_3_csparse", std::string robustCoreName = "Huber", bool structureOnly = false) {
    std::string solverType = solverName;
    robustType_ = robustCoreName;
    structureOnly_ = structureOnly;
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer_.setVerbose(false);
    optimizer_.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solverType, solverProperty));
  }
  /** \brief construct bundle adjustment of slide window pose and local map
   * @param slidewindow --- slide window with keyframes' pose
   * @param fsm  --- feature manager with local map infomations
   * @param sigma --- standard deviation of reproject error at normalized plane
   * @return
   */
  bool constructWindowFrameOptimize(std::vector<FramePtr>& slidewindow,FeatureManager& fsm,double sigma) {
    clearVertexAndEdges();
    if (slidewindow.size() < 2) {
      printf("[BAG2O]:Failure!SlideWindow size %ld is not enough!\n",slidewindow.size());
      return false;
    }
    //normalized plane camera model:focal length = 1.0, principle point = [0.,0.]
    g2o::CameraParameters * cam_params = new g2o::CameraParameters (1.0,Eigen::Vector2d::Zero(),0);
    double informationGain = 1.0 / (sigma * sigma);
    cam_params->setId(0);
    if (!optimizer_.addParameter(cam_params)) {
      printf("[BAG2O]:Failure!Add camera paramter failed!\n");
      return false;
    }

    int vertexId = 0;
    //step-1 : add pose vertex
    for (auto s : slidewindow) {
      g2o::VertexSE3Expmap * v_cam = new g2o::VertexSE3Expmap();
      v_cam->setId(vertexId);
      //Fix first pose for gauge freedom
      if (vertexId == 0) {
        v_cam->setFixed(true);
      }
      Eigen::Matrix3d eRcw;
      Eigen::Vector3d etcw;
      cv::Mat Rcw,tcw;
      s->getInversePose(Rcw,tcw);
      if (Rcw.empty() || tcw.empty()) {
        std::cerr << "[BAG2O]:Failure!This frame pose is not set!" << std::endl;
        return false;
      }
      cv::cv2eigen(Rcw,eRcw);
      cv::cv2eigen(tcw,etcw);
      g2o::SE3Quat Tcw(eRcw,etcw);
      v_cam->setEstimate(Tcw);
      optimizer_.addVertex(v_cam);
      poseId_[s] = vertexId;
      vertexId++;
    }
    //step-2: add landmark vertex
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (std::map<uint64_t,Feature>::const_iterator it = features.begin();it != features.end();it++) {
      const Feature& fea = it->second;
      const uint64_t idx = it->first;
      if (!fea.isReadyForOptimize()) {
        continue;
      }
      cv::Point3f ft3d = fea.getPts3DInWorld();
      g2o::VertexPointXYZ *v_p = new g2o::VertexPointXYZ();
      v_p->setId(vertexId);
      if (fea.getGoodCount() >= 3) {
        v_p->setFixed(true);
      }
      v_p->setEstimate(Eigen::Vector3d(ft3d.x,ft3d.y,ft3d.z));
      v_p->setMarginalized(true);
      optimizer_.addVertex(v_p);
      featId_[idx] = vertexId;
      vertexId++;
    }

    //step-3: add reproject error edge
    for (auto s : slidewindow) {
      if (!poseId_.count(s)) {
        continue;
      }
      int poseVId = poseId_[s];
      for (std::map<uint64_t,int>::const_iterator it = featId_.begin(); it != featId_.end(); it++) {
        uint64_t idx = it->first;
        int featVId = it->second;
        if (!features.count(idx)) {
          continue;
        }
        PixelCoordinate uv;
        if (features[idx].getPixelInFrame(s,uv)) {
          cv::Point2f z = uv.second;
          g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
          //check edgeProjectXYZ2UV for determining id of vertex connecting with edge
          edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(featVId)));
          edge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(poseVId)));
          edge->setMeasurement(Eigen::Vector2d(z.x,z.y));
          edge->setInformation(Eigen::Matrix2d::Identity() * informationGain);
          edge->setRobustKernel(new g2o::RobustKernelHuber());
          edge->setParameterId(0,0);
          optimizer_.addEdge(edge);
        }
      }
    }
    //step-4: optimization
    optimizer_.setVerbose(false);
    optimizer_.initializeOptimization();
    optimizer_.optimize(5);
    return true;
  }
  /** \brief update pose of slide window and coordinate of local map
   *
   * @param slidewindow --- slide window of keyframe
   * @param fsm --- feature manager
   */
  void updatePoseAndMap(std::vector<FramePtr>& slidewindow,FeatureManager& fsm) {
    for (auto f : slidewindow) {
      if (!poseId_.count(f)) {
        continue;
      }
      int poseVId = poseId_[f];
      g2o::VertexSE3Expmap* poseV = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer_.vertex(poseVId));
      Eigen::Matrix3d Rwc = poseV->estimate().rotation().toRotationMatrix().transpose();
      Eigen::Vector3d twc = -Rwc * poseV->estimate().translation();
      cv::Mat cRwc,ctwc;
      cv::eigen2cv(Rwc, cRwc);
      cv::eigen2cv(twc, ctwc);
      f->setPoseInWorld(cRwc,ctwc);
    }
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (auto c : featId_) {
      uint64_t cornerId = c.first;
      int featVId = c.second;
      if (!features.count(cornerId)) {
        continue;
      }
      g2o::VertexPointXYZ* pointV = dynamic_cast<g2o::VertexPointXYZ*> (optimizer_.vertex(featVId));
      Eigen::Vector3d pt3d = pointV->estimate();
      cv::Point3f pt3old = features[cornerId].getPts3DInWorld();
      features[cornerId].setPtsInWorld(cv::Point3f(pt3d.x(),pt3d.y(),pt3d.z()));
    }
  }

  /** \brief clear vertex and edges for next optimization
   *
   */
  void clearVertexAndEdges() {
    optimizer_.clear();
    optimizer_.clearParameters();
    poseId_.clear();
    featId_.clear();
  }

 private:
  g2o::SparseOptimizer optimizer_;
  std::string robustType_;
  bool structureOnly_;
  std::map<FramePtr,int> poseId_;
  std::map<uint64_t,int> featId_;
};