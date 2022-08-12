//
// Created by kdq on 2021/6/4.
//
#pragma once
#include <chrono>
#include <random>
#include <Eigen/Core>
#include "sophus/se3.hpp"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel_impl.h"
#include "opencv2/core/eigen.hpp"
#include "Estimator.hpp"

using namespace vio;


//顶点的定义:设置vertex，传入的参数包括该vertex的自由度和类型
//需要覆盖虚函数setToOriginImpl和oplusImpl，用于提供vertex的设置原点和更新方法
class PoseVertex : public g2o::BaseVertex<6,Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PoseVertex() {
    fx_ = 1.0;
    fy_ = 1.0;
    ux_ = 0.;
    uy_ = 0.;
  };
  PoseVertex(double fx,double fy,double ux,double uy) {
    fx_ = fx;
    fy_ = fy;
    ux_ = ux;
    uy_ = uy;
  };
  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }
  virtual bool read(std::istream& is) {};
  virtual bool write(std::ostream& os) const {};
  double fx_,fy_,ux_,uy_;
};

class PointVertex : public g2o::BaseVertex<3,Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PointVertex() {};
  virtual void setToOriginImpl() override {
    _estimate.setZero();
  }
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update[0],update[1],update[2]);
  }
  virtual bool read(std::istream& is) {};
  virtual bool write(std::ostream& os) const {};

};

//边的定义：集成二元边，需要指定边也就是观测的自由度和类型，以及该边连接的两个定点的类型
//需要覆盖函数computeError指定误差计算的方法，而linearizeOplus函数可以覆盖用于指定误差相对于顶点的雅可比，也可以不覆盖则默认会使用数值微分的方法
class ReprojectUVEdge : public g2o::BaseBinaryEdge<2,Eigen::Vector2d,PoseVertex,PointVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ReprojectUVEdge(){};

  virtual void computeError() override{
    const PoseVertex* v1 = static_cast<const PoseVertex*>(_vertices[0]);
    const PointVertex* v2 = static_cast<const PointVertex*>(_vertices[1]);
    Eigen::Vector3d ptInCam =  v1->estimate() * v2->estimate() / v2->estimate().z();
    Eigen::Vector2d ptInUV;
    ptInUV.x() = v1->fx_ * ptInCam.x() + v1->ux_;
    ptInUV.y() = v1->fy_ * ptInCam.y() + v1->uy_;
    _error = ptInUV  - _measurement;
  }

  virtual void linearizeOplus() override{
    const PoseVertex* poseV = static_cast<PoseVertex *> (_vertices[0]);
    const PointVertex * pointV = static_cast<PointVertex *> (_vertices[1]);
    Eigen::Vector3d pc =  poseV->estimate() * pointV->estimate();
    double fx = poseV->fx_;
    double fy = poseV->fy_;
    double x = pc.x();
    double y = pc.y();
    double z = pc.z();
    double z2 = z*z;
    Eigen::Matrix<double,2,3> Jac_uv2normalized;
    Jac_uv2normalized << fx/z, 0., -fx*x/z2, 0., fy/z, -fy*y/z2;
    _jacobianOplusXi << fx/z,  0,    -fx*x/z2,    -fx*x*y/z2, fx + fx*x*x/z2, -fx*y/z,
                          0,  fy/z,  -fy*y/z2,  -fy-fy*y*y/z2,    fy*x*y/z2,  fy*x/z;
    _jacobianOplusXj = Jac_uv2normalized * poseV->estimate().rotationMatrix();
  }

  virtual bool read(std::istream& is) {};
  virtual bool write(std::ostream& os) const {};

};


class ReprojectUVOnlyPoseEdge : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,PoseVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ReprojectUVOnlyPoseEdge(Eigen::Vector3d point3d) {
    points_ = point3d;
  };
  virtual void computeError() override {
    const PoseVertex *v1 = static_cast<const PoseVertex *>(_vertices[0]);
    Eigen::Vector3d ptInCam = v1->estimate() * points_ / points_.z();
    Eigen::Vector2d ptInUV;
    ptInUV.x() = v1->fx_ * ptInCam.x() + v1->ux_;
    ptInUV.y() = v1->fy_ * ptInCam.y() + v1->uy_;
    _error = ptInUV - _measurement;
  }
//  virtual void linearizeOplus() override {
//    const PoseVertex *poseV = static_cast<PoseVertex *> (_vertices[0]);
//    Eigen::Vector3d pc = poseV->estimate() * points_;
//    double fx = poseV->fx_;
//    double fy = poseV->fy_;
//    double x = pc.x();
//    double y = pc.y();
//    double z = pc.z();
//    double z2 = z*z;
//    Eigen::Matrix<double,2,3> Jac_uv2normalized;
//    Jac_uv2normalized << fx/z, 0., -fx*x/z2, 0., fy/z, -fy*y/z2;
//    _jacobianOplusXi << fx/z,  0,    -fx*x/z2,    -fx*x*y/z2, fx + fx*x*x/z2, -fx*y/z,
//                           0,  fy/z,  -fy*y/z2,  -fy-fy*y*y/z2,    fy*x*y/z2,  fy*x/z;
//  }
  virtual bool read(std::istream &is) {};
  virtual bool write(std::ostream &os) const {};

 private:
  Eigen::Vector3d points_;
};

class BundleAdjustmentByG2O {
 public:
  BundleAdjustmentByG2O() {
//    std::cout << "G2O优化： \n 1、定义好顶点:包括顶点的自由度、数据类型以及设置原点的方法；\n"
//              << "            2、定义边：包括边的自由度、类型以及边相连顶点的类型和计算误差的方法以及误差相对于顶点的雅可比（该部分可以不定义，若不定义则默认使用数值微分方式);\n"
//              << "            3、注意边中顶点的顺序，在边添加顶点setVertex(vertexId,vertextype),指定顶点位置的时候需要和边定义保持一致；\n"
//              << "            4、注意边需要setInformation(...），否则初始化为0矩阵，则无法优化出正确状态;\n"
//              << "            5、注意当只有一个pose顶点，且setFixed(true)，而其他point顶点不可以设置setMarginalized(true)，否则将报段错误：原因是优先边缘化所有点是要计算pose顶点，而pose顶点fixed是无法求得的，如果有两个pose顶点，则可以只设置一个fixed，这样point顶点是可以设置边缘化的；\n"
//              << "            6、若不定义误差相对于顶点的雅可比使用自动数值微分似乎有点耗时；"
//              << std::endl;

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    g2o::OptimizationAlgorithm * algrithm = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer_.setAlgorithm(algrithm);
  }

  void singleFrameOptimize(FramePtr& f,FeatureManager& fsm) {
    if (f == nullptr) {
      std::cerr << "[BA]:Failure!Frame is nullptr!" << std::endl;
      return;
    }
    Eigen::Matrix3d eRcw;
    Eigen::Vector3d etcw;
    cv::Mat Rcw,tcw;
    f->getInversePose(Rcw,tcw);
    if (Rcw.empty() || tcw.empty()) {
      std::cerr << "[BA]:Failure!This frame pose is not set!" << std::endl;
      return;
    }
    cv::cv2eigen(Rcw,eRcw);
    cv::cv2eigen(tcw,etcw);
    PoseVertex *pVertex = new PoseVertex();
    pVertex->setId(0);
    pVertex->setEstimate(Sophus::SE3d(eRcw,etcw));
    optimizer_.addVertex(pVertex);

    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    std::map<uint64_t,cv::Point2f>& corners = f->getCorners();
    int matchSize = 0;
    for(auto c : corners) {
      uint64_t cornerId = c.first;
      if (!features.count(cornerId)) {
        continue;
      }

      if (!features[cornerId].isReadyForOptimize()) {
        continue;
      }
      PixelCoordinate uv;
      if (features[cornerId].getPixelInFrame(f,uv)) {
        cv::Point2f normalizePt = uv.second;
        cv::Point3f ptsW = features[cornerId].getPts3DInWorld();
        ReprojectUVOnlyPoseEdge *edge = new ReprojectUVOnlyPoseEdge(Eigen::Vector3d(ptsW.x,ptsW.y,ptsW.z));
        edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer_.vertex(0)));
        edge->setMeasurement(Eigen::Vector2d(normalizePt.x,normalizePt.y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer_.addEdge(edge);
        matchSize++;
      }
    }
    if (matchSize < 5) {
      std::cout << "[BA]:match size is less than 5,so shut down ba!" << std::endl;
      return;
    }
    std::cout<<"开始优化"<< std::endl;
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
    std::cout<<"优化完毕"<<std::endl;
  }
  bool windowFrameOptimize(std::vector<FramePtr>& slidewindow,FeatureManager& fsm) {
    if (slidewindow.size() < 2) {
      printf("[G2O-AddVertex]:Failure!SlideWindow size %ld is not enough!\n",slidewindow.size());
      return false;
    }
    //add pose vertex
    int vertexId = 0;
    for (auto s : slidewindow) {
      PoseVertex *poseV = new PoseVertex();
      poseV->setId(vertexId);
      if (vertexId == 0) {
        poseV->setFixed(true);
      }
      Eigen::Matrix3d eRcw;
      Eigen::Vector3d etcw;
      cv::Mat Rcw,tcw;
      s->getInversePose(Rcw,tcw);
      if (Rcw.empty() || tcw.empty()) {
        std::cerr << "[BA]:Failure!This frame pose is not set!" << std::endl;
        return false;
      }
      cv::cv2eigen(Rcw,eRcw);
      cv::cv2eigen(tcw,etcw);
      std::cout << "PoseVertexId = " << vertexId << " Ptr: " << s << " time: " << s->timestamp_ << " Pose: " << etcw.transpose() << std::endl;
      poseV->setId(vertexId);
      poseV->setEstimate(Sophus::SE3d(eRcw,etcw));
      optimizer_.addVertex(poseV);
      poseId_[s] = vertexId;
      vertexId++;
    }
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (std::map<uint64_t,Feature>::const_iterator it = features.begin();it != features.end();it++) {
      const Feature& fea = it->second;
      if (!fea.isReadyForOptimize()) {
        continue;
      }
      cv::Point3f ft3d = fea.getPts3DInWorld();
      PointVertex *pointV = new PointVertex();
      pointV->setId(vertexId);
      pointV->setEstimate(Eigen::Vector3d(ft3d.x,ft3d.y,ft3d.z));
      pointV->setMarginalized(true);
      optimizer_.addVertex(pointV);
      featId_[it->first] = vertexId;
      vertexId++;
    }
    for (auto s : slidewindow) {
      if (!poseId_.count(s)) {
        continue;
      }
      int poseVId = poseId_[s];
      for (std::map<uint64_t,int>::const_iterator it = featId_.begin(); it != featId_.end(); it++) {
        uint64_t featId = it->first;
        int featVId = it->second;
        if (!features.count(featId)) {
          continue;
        }
        PixelCoordinate uv;
        if (features[featId].getPixelInFrame(s,uv)) {
          cv::Point2f normUV = uv.second;
          ReprojectUVEdge *edge = new ReprojectUVEdge();
          edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer_.vertex(poseVId)));
          edge->setVertex(1,dynamic_cast<PointVertex*>(optimizer_.vertex(featVId)));
          std::cout << "Frame: " << s  << " poseVId: " << poseVId
                    << " FeatId: " << featId << " FeatVId:" << featVId
                    << " UV: " << uv.first << " NormUV: " << normUV <<std::endl;
          edge->setMeasurement(Eigen::Vector2d(normUV.x,normUV.y));
          edge->setInformation( Eigen::Matrix2d::Identity());
          edge->setRobustKernel(new g2o::RobustKernelHuber());
          optimizer_.addEdge(edge);
        }
      }
    }
    std::cout<<"开始优化"<< std::endl;
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
    std::cout<<"优化完毕"<<std::endl;
    return true;
  }

  bool windowFrameOnlyPoseOptimize(std::vector<FramePtr>& slidewindow,FeatureManager& fsm) {
    if (slidewindow.size() < 2) {
      printf("[G2O-AddVertex]:Failure!SlideWindow size %ld is not enough!\n",slidewindow.size());
      return false;
    }
    //add pose vertex
    int vertexId = 0;
    for (auto s : slidewindow) {
      PoseVertex *poseV = new PoseVertex();
      poseV->setId(vertexId);
      if (vertexId == 0) {
        poseV->setFixed(true);
      }
      Eigen::Matrix3d eRcw;
      Eigen::Vector3d etcw;
      cv::Mat Rcw,tcw;
      s->getInversePose(Rcw,tcw);
      if (Rcw.empty() || tcw.empty()) {
        std::cerr << "[BA]:Failure!This frame pose is not set!" << std::endl;
        return false;
      }
      cv::cv2eigen(Rcw,eRcw);
      cv::cv2eigen(tcw,etcw);
      poseV->setId(vertexId);
      poseV->setEstimate(Sophus::SE3d(eRcw,etcw));
      optimizer_.addVertex(poseV);
      poseId_[s] = vertexId;
      vertexId++;
    }
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (auto s : slidewindow) {
      if (!poseId_.count(s)) {
        continue;
      }
      int poseVId = poseId_[s];
      for (std::map<uint64_t,Feature>::iterator it = features.begin(); it != features.end(); it++) {
        if (!it->second.isReadyForOptimize()) {
          continue;
        }
        PixelCoordinate uv;

        if (it->second.getPixelInFrame(s,uv)) {
          cv::Point2f normUV = uv.second;
          cv::Point3f pt3d = it->second.getPts3DInWorld();
          ReprojectUVOnlyPoseEdge *edge = new ReprojectUVOnlyPoseEdge(Eigen::Vector3d(pt3d.x,pt3d.y,pt3d.z));
          edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer_.vertex(poseVId)));
          edge->setMeasurement(Eigen::Vector2d(normUV.x,normUV.y));
          edge->setInformation( 10000*Eigen::Matrix2d::Identity());
          edge->setRobustKernel(new g2o::RobustKernelHuber());
          optimizer_.addEdge(edge);
        }
      }
    }
    std::cout<<"开始优化"<< std::endl;
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(40);
    std::cout<<"优化完毕"<<std::endl;
    return true;
  }


  bool windowFrameOptimizeUV(std::vector<FramePtr>& slidewindow,FeatureManager& fsm) {
    if (slidewindow.size() < 2) {
      printf("[G2O-AddVertex]:Failure!SlideWindow size %ld is not enough!\n",slidewindow.size());
      return false;
    }
    //add pose vertex
    int vertexId = 0;
    for (auto s : slidewindow) {
      PoseVertex *poseV = new PoseVertex(s->cam_->fx(),s->cam_->fy(),s->cam_->cx(),s->cam_->cy());
      poseV->setId(vertexId);
      if (vertexId == 0) {
        poseV->setFixed(true);
      }
      Eigen::Matrix3d eRcw;
      Eigen::Vector3d etcw;
      cv::Mat Rcw,tcw;
      s->getInversePose(Rcw,tcw);
      if (Rcw.empty() || tcw.empty()) {
        std::cerr << "[BA]:Failure!This frame pose is not set!" << std::endl;
        return false;
      }
      cv::cv2eigen(Rcw,eRcw);
      cv::cv2eigen(tcw,etcw);
     // std::cout << "PoseVertexId = " << vertexId << " Ptr: " << s << " time: " << s->timestamp_ << " Pose: " << etcw.transpose() << std::endl;
      poseV->setId(vertexId);
      poseV->setEstimate(Sophus::SE3d(eRcw,etcw));
      optimizer_.addVertex(poseV);
      poseId_[s] = vertexId;
      vertexId++;
    }
    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
    for (std::map<uint64_t,Feature>::const_iterator it = features.begin();it != features.end();it++) {
      const Feature& fea = it->second;
      if (!fea.isReadyForOptimize()) {
        continue;
      }
      cv::Point3f ft3d = fea.getPts3DInWorld();
      PointVertex *pointV = new PointVertex();
      pointV->setId(vertexId);
      pointV->setEstimate(Eigen::Vector3d(ft3d.x,ft3d.y,ft3d.z));
      pointV->setMarginalized(true);
      optimizer_.addVertex(pointV);
      featId_[it->first] = vertexId;
      vertexId++;
    }
    for (auto s : slidewindow) {
      if (!poseId_.count(s)) {
        continue;
      }
      int poseVId = poseId_[s];
      for (std::map<uint64_t,int>::const_iterator it = featId_.begin(); it != featId_.end(); it++) {
        uint64_t featId = it->first;
        int featVId = it->second;
        if (!features.count(featId)) {
          continue;
        }
        PixelCoordinate uv;
        if (features[featId].getPixelInFrame(s,uv)) {
          cv::Point2f corner = uv.first;
          ReprojectUVEdge *edge = new ReprojectUVEdge();
          edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer_.vertex(poseVId)));
          edge->setVertex(1,dynamic_cast<PointVertex*>(optimizer_.vertex(featVId)));
//          std::cout << "Frame: " << s  << " poseVId: " << poseVId
//                    << " FeatId: " << featId << " FeatVId:" << featVId
//                    << " UV: " << corner << " NormUV: " << uv.second <<std::endl;
          edge->setMeasurement(Eigen::Vector2d(corner.x,corner.y));
          edge->setInformation( Eigen::Matrix2d::Identity());
          edge->setRobustKernel(new g2o::RobustKernelCauchy());
          optimizer_.addEdge(edge);
        }
      }
    }
    std::cout<<"开始优化"<< std::endl;
    optimizer_.setVerbose(false);
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
    std::cout<<"优化完毕"<<std::endl;
    return true;
  }

  void updatePoseAndMap(std::vector<FramePtr>& slidewindow,FeatureManager& fsm) {
    for (auto f : slidewindow) {
      if (!poseId_.count(f)) {
        continue;
      }
      int poseVId = poseId_[f];
      PoseVertex* poseV = dynamic_cast<PoseVertex*>(optimizer_.vertex(poseVId));
      Eigen::Matrix3d Rwc = poseV->estimate().rotationMatrix().transpose();
      Eigen::Vector3d twc = -poseV->estimate().rotationMatrix().transpose() * poseV->estimate().translation();
      cv::Mat cRwc,ctwc;
      cv::eigen2cv(Rwc, cRwc);
      cv::eigen2cv(twc, ctwc);
      cv::Mat Rwcold,twcold;
      f->getPoseInWorld(Rwcold,twcold);
      std::cout << "Optimization : \n" << "RwcOld : \n" << Rwcold << "RwcNew : \n" << cRwc << std::endl;
      std::cout <<  "twcOld : " << twcold.t() << "twcNew : " << ctwc.t() << std::endl;
      f->setPoseInWorld(cRwc,ctwc);
    }
//    std::map<uint64_t,Feature>& features = fsm.getFeatureMap();
//    for (auto c : featId_) {
//      uint64_t cornerId = c.first;
//      int featVId = c.second;
//      if (!features.count(cornerId)) {
//        continue;
//      }
//      PointVertex* pointV = dynamic_cast<PointVertex*> (optimizer_.vertex(featVId));
//      Eigen::Vector3d pt3d = pointV->estimate();
//      cv::Point3f pt3old = features[cornerId].getPts3DInWorld();
//      std::cout << "Optimization: old feat = " << pt3old << " new feat = " << pt3d.transpose() << std::endl;
//      features[cornerId].setPtsInWorld(cv::Point3f(pt3d.x(),pt3d.y(),pt3d.z()));
//    }
  }



  void testTcw() {
    std::vector<Eigen::Vector2d> meas1,meas2;
    std::vector<Eigen::Vector3d> pt3ds,pts3dsNoise;
    std::random_device sd;
    std::mt19937_64 genorator(sd());
    std::uniform_real_distribution<float> noise(-0.1,0.1);
    std::uniform_real_distribution<float> pt(-2,2);

    Eigen::Matrix3d R1;
    R1.setIdentity();
    Eigen::AngleAxisd angaxi(M_PI/2,Eigen::Vector3d(0,0,1).normalized());
    Eigen::Matrix3d R2 = angaxi.toRotationMatrix();
    std::cout << "R2 = " << R2 << std::endl;
    Eigen::Vector3d t1;
    t1.setZero();
    Eigen::Vector3d t2(0.1,0.3,0);
    for (size_t i = 0; i < 20; i++) {
      Eigen::Vector3d point3d(pt(genorator),pt(genorator),1);
      Eigen::Vector3d meas3d1 = R1 * point3d + t1;
      Eigen::Vector2d meas2d1 = meas3d1.head(2)/meas3d1.z();
      Eigen::Vector3d meas3d2 = R2 * point3d + t2;
      Eigen::Vector2d meas2d2 = meas3d2.head(2) / meas3d2.z();
      pt3ds.push_back(point3d);
      pts3dsNoise.push_back(point3d + 4 * Eigen::Vector3d(noise(genorator),noise(genorator),noise(genorator)));
      meas1.push_back(meas2d1 + 0. * Eigen::Vector2d(noise(genorator),noise(genorator)));
      meas2.push_back(meas2d2 + 0. * Eigen::Vector2d(noise(genorator),noise(genorator)));
    }
    //add pose vertex
    PoseVertex * posevertex1 = new PoseVertex();
    posevertex1->setId(0);
    Sophus::SE3d se30 = Sophus::SE3d();
    posevertex1->setEstimate(se30);
    //注意如果set true，除非有其他的定点是非fixed，否则后面的点是不可以setMarginalized(true)的
    posevertex1->setFixed(true);
    optimizer_.addVertex(posevertex1);

    PoseVertex * posevertex2 = new PoseVertex();
    posevertex2->setId(1);
    posevertex2->setEstimate(Sophus::SE3d(R2, t2 + Eigen::Vector3d(0.1,0.05,0.01)));
    //注意如果set true，后面的点是不可以setMarginalized(true)的
    optimizer_.addVertex(posevertex2);

    //add point vertex
    for (size_t i = 0; i < 20; i++) {
      PointVertex * pointvertex = new PointVertex();
      pointvertex->setId(i + 2);
      pointvertex->setEstimate(pts3dsNoise[i]);
      pointvertex->setMarginalized(true);
      optimizer_.addVertex(pointvertex);
    }

    for (int i = 0; i < 20; ++i) {
      ReprojectUVEdge *edge = new ReprojectUVEdge();
      edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer_.vertex(0)));
      edge->setVertex(1,dynamic_cast<PointVertex*>(optimizer_.vertex(i + 2)));
      edge->setMeasurement(meas1[i]);
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(new g2o::RobustKernelCauchy());
      optimizer_.addEdge(edge);
    }

    for (int i = 0; i < 20; ++i) {
      ReprojectUVEdge *edge = new ReprojectUVEdge();
      edge->setVertex(0,dynamic_cast<PoseVertex*>(optimizer_.vertex(1)));
      edge->setVertex(1,dynamic_cast<PointVertex*>(optimizer_.vertex(i + 2)));
      edge->setMeasurement(meas2[i]);
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber());
      optimizer_.addEdge(edge);
    }
    std::chrono::steady_clock::time_point time1 = std::chrono::steady_clock::now();
    optimizer_.setVerbose(true);
    optimizer_.initializeOptimization();
    optimizer_.optimize(10);
    std::chrono::steady_clock::time_point time2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time2 - time1);

    std::cout << "Optimize by g2o cost time =  " << time_used.count() << std::endl;
    std::cout << static_cast<PoseVertex*>(optimizer_.vertex(0))->estimate().matrix() << std::endl;
    std::cout << static_cast<PoseVertex*>(optimizer_.vertex(1))->estimate().matrix() << std::endl;

    for (size_t i = 0; i < 20; i++) {
      double errInitPts = (pts3dsNoise[i] - pt3ds[i]).norm();
      double errEstPts = (static_cast<PointVertex*>(optimizer_.vertex(i + 2))->estimate() - pt3ds[i]).norm();
      std::cout <<  " InitPtsErr : " << errInitPts << " EstPtsErr : " << errEstPts << std::endl;
    }
  }


 private:
  g2o::SparseOptimizer optimizer_;
  std::map<FramePtr,int> poseId_;
  std::map<uint64_t,int> featId_;
};

