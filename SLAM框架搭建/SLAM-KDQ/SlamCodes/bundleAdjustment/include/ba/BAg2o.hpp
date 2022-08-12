//
// Created by kdq on 2021/6/10.
//

#ifndef BUNDLEADJUSTMENT_BAG2O_HPP
#define BUNDLEADJUSTMENT_BAG2O_HPP
#include <iostream>
#include <unordered_set>
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
class BAG2O {
 public:
  BAG2O(std::string solverName,std::string robustCoreName,bool structureOnly = false) {
    std::string solverType = solverName;
    robustType_ = robustCoreName;
    structureOnly_ = structureOnly;
    if (solverName.empty()) {
      solverType = "lm_fix6_3_cholmod";
    }
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer_.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solverType,solverProperty));
  }

  void test(double OUTLIER_RATIO,double PIXEL_NOISE,bool STRUCTURE_ONLY) {
    std::vector<Eigen::Vector3d> true_points;
    for (size_t i=0;i<500; ++i)
    {
      true_points.push_back(Eigen::Vector3d((g2o::Sampler::uniformRand(0., 1.)-0.5)*3,
                                     g2o::Sampler::uniformRand(0., 1.)-0.5,
                                     g2o::Sampler::uniformRand(0., 1.)+3));
    }

    double focal_length= 1000.;
    Eigen::Vector2d principal_point(320., 240.);

    std::vector<g2o::SE3Quat,Eigen::aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer_.addParameter(cam_params)) {
      assert(false);
    }

    int vertex_id = 0;
    for (size_t i=0; i<15; ++i) {
      Eigen::Vector3d trans(i*0.04-1.,0,0);
      Eigen:: Quaterniond q;
      q.setIdentity();
      g2o::SE3Quat pose(q,trans);
      g2o::VertexSE3Expmap * v_se3
        = new g2o::VertexSE3Expmap();
      v_se3->setId(vertex_id);
      if (i<2){
        v_se3->setFixed(true);
      }
      v_se3->setEstimate(pose);
      optimizer_.addVertex(v_se3);
      true_poses.push_back(pose);
      vertex_id++;
    }
    int point_id=vertex_id;
    int point_num = 0;
    double sum_diff2 = 0;

    std::cout << std::endl;
    std::unordered_map<int,int> pointid_2_trueid;
    std::unordered_set<int> inliers;

    for (size_t i=0; i<true_points.size(); ++i){
      g2o::VertexPointXYZ * v_p
        = new g2o::VertexPointXYZ();
      v_p->setId(point_id);
      v_p->setMarginalized(true);
      v_p->setEstimate(true_points.at(i)
                       + Eigen::Vector3d(g2o::Sampler::gaussRand(0., 1),
                                  g2o::Sampler::gaussRand(0., 1),
                                  g2o::Sampler::gaussRand(0., 1)));
      int num_obs = 0;
      for (size_t j=0; j<true_poses.size(); ++j){
        Eigen::Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
        if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
          ++num_obs;
        }
      }
      if (num_obs>=2){
        optimizer_.addVertex(v_p);
        bool inlier = true;
        for (size_t j=0; j<true_poses.size(); ++j){
          Eigen::Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
          if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
            double sam = g2o::Sampler::uniformRand(0., 1.);
            if (sam < OUTLIER_RATIO){
              z = Eigen::Vector2d(g2o::Sampler::uniformRand(0, 640),
                                  g2o::Sampler::uniformRand(0, 480));
              inlier= false;
            }
            z += Eigen::Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                          g2o::Sampler::gaussRand(0., PIXEL_NOISE));
            g2o::EdgeProjectXYZ2UV * e
              = new g2o::EdgeProjectXYZ2UV();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
            (optimizer_.vertices().find(j)->second));
            e->setMeasurement(z);
            e->information() = Eigen::Matrix2d::Identity();
            if (robustType_.empty()) {
              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
            }
            e->setParameterId(0, 0);
            optimizer_.addEdge(e);
          }
        }

        if (inlier){
          inliers.insert(point_id);
          Eigen::Vector3d diff = v_p->estimate() - true_points[i];

          sum_diff2 += diff.dot(diff);
        }
        pointid_2_trueid.insert(std::make_pair(point_id,i));
        ++point_id;
        ++point_num;
      }
    }
    std::cout << std::endl;
    optimizer_.initializeOptimization();
    optimizer_.setVerbose(true);
    if (structureOnly_){
      g2o::StructureOnlySolver<3> structure_only_ba;
      std::cout << "Performing structure-only BA:"   << std::endl;
      g2o::OptimizableGraph::VertexContainer points;
      for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
        if (v->dimension() == 3)
          points.push_back(v);
      }
      structure_only_ba.calc(points, 10);
    }
    //optimizer.save("test.g2o");
    std::cout << std::endl;
    std::cout << "Performing full BA:" << std::endl;
    optimizer_.optimize(10);
    std::cout << std::endl;
    std::cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << std::endl;
    point_num = 0;
    sum_diff2 = 0;
    for (std::unordered_map<int,int>::iterator it=pointid_2_trueid.begin();
         it!=pointid_2_trueid.end(); ++it){
      g2o::HyperGraph::VertexIDMap::iterator v_it
        = optimizer_.vertices().find(it->first);
      if (v_it==optimizer_.vertices().end()){
        std::cerr << "Vertex " << it->first << " not in graph!" << std::endl;
        exit(-1);
      }
      g2o::VertexPointXYZ * v_p
        = dynamic_cast< g2o::VertexPointXYZ * > (v_it->second);
      if (v_p==0){
        std::cerr << "Vertex " << it->first << "is not a PointXYZ!" <<  std::endl;
        exit(-1);
      }
      Eigen::Vector3d diff = v_p->estimate()-true_points[it->second];
      if (inliers.find(it->first)==inliers.end())
        continue;
      sum_diff2 += diff.dot(diff);
      ++point_num;
    }
    std::cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << std::endl;
    std::cout << std::endl;
  }

 private:
  g2o::SparseOptimizer optimizer_;
  std::string robustType_;
  bool structureOnly_;
};



#endif //BUNDLEADJUSTMENT_BAG2O_HPP
