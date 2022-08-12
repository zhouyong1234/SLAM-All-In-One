/*
 * @Author: your name
 * @Date: 2021-04-02 10:37:48
 * @LastEditTime: 2021-10-09 22:00:36
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/src/BackEnd/Optimization.cc
 */

#include <cmath>
#include <iostream>
#include <assert.h>
#include <chrono>
#include <cstdlib> // 随机数

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "DataPretreat/Config.h"
#include "BackEnd/Optimization.h"
#include "FrontEnd/PoseSolver.h"

// #define DEBUG
#ifdef DEBUG
    #define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
    #define CHECK_INFO_2(x,y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
    #define CHECK_INFO(x) //std::cout << x << std::endl;
    #define CHECK_INFO_2(x,y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

Optimization::Optimization(Config &config): setting(config)
{
    // std::cout << "[INFO] Optimization Start." << std::endl;
}

Optimization::~Optimization()
{
}


void Optimization::Kneip_Ransac(std::vector<cv::Point3f> &PointsInWorldVec_0
    , std::vector<cv::Point3f> &PointsInPixelVec_1
    , std::vector<int> &Interior
    , Eigen::Matrix4f &Pose )
{
    assert(PointsInWorldVec_0.size() == PointsInPixelVec_1.size());
    assert(PointsInWorldVec_0.size() > 4);

    unsigned seed = time(0);
    srand(seed);

    std::vector<int> InlierNumVec;
    std::vector< std::vector<int> > InlierIDVec;
    std::vector< Eigen::Matrix4f > PoseVec;

    PoseSolver ps(PointsInWorldVec_0, PointsInPixelVec_1, setting);

    for (size_t i(0); i<setting.oc.maxRansacIter; ++i){

        std::set<int> idSet;
        std::vector<int> idVec;
        while( idSet.size() < 4 ){
            int rand_num = rand()%(PointsInWorldVec_0.size());
            idSet.insert(rand_num);
        }

        for (std::set<int>::iterator iter = idSet.begin(); iter!=idSet.end(); ++iter){
            idVec.push_back(*iter);
        }
        assert(idVec.size()==4);


        // ps.YuPnP(idVec, PointsInWorldVec_0, PointsInPixelVec_1);
        ps.KneipPnP(idVec, PointsInWorldVec_0, PointsInPixelVec_1);
        Eigen::Matrix4f T12 = ps.GetT12();
        PoseVec.emplace_back(T12);

        std::vector<int> tmp;
        int cnt = 0;
        for(size_t j(0); j<PointsInPixelVec_1.size();++j)
        {
            double error = ps.ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, j, T12);
            // CHECK_INFO_2("error : ", error);
            if(error < setting.oc.ErrorTH){
                cnt++;
                tmp.push_back(j);
            }
        }
        InlierIDVec.emplace_back(tmp);
        InlierNumVec.emplace_back(cnt);
    }

    std::vector<int>::iterator maxInfo = max_element(InlierNumVec.begin(),InlierNumVec.end());
    int max_pos = std::distance(InlierNumVec.begin(), maxInfo);
    Pose = PoseVec[max_pos];
    for(size_t k(0); k<InlierIDVec[max_pos].size();++k){
        int id = InlierIDVec[max_pos][k];
        Interior[id] = 1;
    }

    // CHECK_INFO_2("max id: ", max_pos);

}


void Optimization::BA_OptimizePose(std::vector<cv::Point3f> &points_3d
    , std::vector<cv::Point3f> &features
    , Eigen::Matrix4f &Pose){

    // 定义优化器输入输出大小
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    // typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // 位姿优化维度为 6, 特征点/路标优化维度为 3
    // 线性方程求解器
    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    // 矩阵块求解器
    // Block* solver_ptr = new Block ( linearSolver );
    // 优化方法：LM
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // 初始化优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 设置图优化的顶点0：位姿
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    // 待优化顶点id=0
    pose->setId ( 0 );
    // 填入待优化变量
    Eigen::Matrix3d R;
    R << Pose( 0,0 ), Pose( 0,1 ), Pose( 0,2 ),
         Pose( 1,0 ), Pose( 1,1 ), Pose( 1,2 ),
         Pose( 2,0 ), Pose( 2,1 ), Pose( 2,2 );
    Eigen::Vector3d t(Pose(0), Pose(1), Pose(2));
    pose->setEstimate ( g2o::SE3Quat ( R,t) );
    optimizer.addVertex ( pose );

    // 设置图优化的顶点1-n：路标的坐标
    int index = 1;
    for ( auto &p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        // g2o把特征点设置为marginalize，是把变量分为相机位姿和特征点两部分，
        //  然后通过上面的方法先算相机位姿增量，再算特征点位姿增量
        point->setMarginalized ( true );
        optimizer.addVertex ( point );
    }

    // 相机内参矩阵
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        static_cast<double>(setting.ip.fx), Eigen::Vector2d ( setting.ip.cx, setting.ip.cy ), 0 );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // 设置图优化的边
    index = 1;
    for ( auto &p:features )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        // 和边链接
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        // 传入测量量
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        // // 设置鲁棒核函数防止外点干扰
        // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        // edge->setRobustKernel(rk);//Huber鲁棒核函数
        // rk->setDelta(thHuberMono);
        // optimizer.addEdge(edge);//添加边
        index++;
    }


    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.setVerbose ( false ); // 输出显示
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2-t1 );

    #ifdef DEBUG
        std::cout<< "[INFO] Optimization costs time: "<<time_used.count() <<" seconds."<<std::endl;
    #endif

    // Eigen::Isometry3d 为变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d( pose->estimate() );
    // Pose = (Eigen::Matrix4d(T)).cast<float>();
    Pose(0,0) = T(0,0);
    Pose(1,0) = T(1,0);
    Pose(2,0) = T(2,0);
    Pose(0,1) = T(0,1);
    Pose(1,1) = T(1,1);
    Pose(2,1) = T(2,1);
    Pose(0,2) = T(0,2);
    Pose(1,2) = T(1,2);
    Pose(2,2) = T(2,2);
    Pose(0,3) = T(0,3);
    Pose(1,3) = T(1,3);
    Pose(2,3) = T(2,3);

    // 取出待优化的点
    for(size_t i=0; i<points_3d.size(); ++i){
        g2o::VertexSBAPointXYZ *v = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+1));
        Eigen::Vector3d pt = v->estimate();
        // std::cout << pt(0) << " " << pt(1) << std::endl;
        // std::cout << points_3d[i] << std::endl;
        points_3d[i] = cv::Point3f( pt(0),pt(1),pt(2) );
    }

}
