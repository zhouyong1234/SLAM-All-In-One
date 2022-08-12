/*
 * @Author: your name
 * @Date: 2021-03-28 17:41:34
 * @LastEditTime: 2021-10-09 19:12:01
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/src/FrontEnd/PoseSolver.cc
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <cassert>
#include <chrono>

// #include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "DataPretreat/Config.h"
#include "FrontEnd/PoseSolver.h"
#include "BackEnd/Optimization.h"

// #define DEBUG
#ifdef DEBUG
    #define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
    #define CHECK_INFO_2(x,y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
    #define CHECK_INFO(x) //std::cout << x << std::endl;
    #define CHECK_INFO_2(x,y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

PoseSolver::PoseSolver(std::vector<cv::Point3f> &coord_0, std::vector<cv::Point3f> &coord_1, Config &config, int initFlag)
    : PointsInPixelVec_0(coord_0),PointsInPixelVec_1(coord_1), setting(config)
{
    PixelPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(0, &PointsInPixelVec_0));
    PixelPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(1, &PointsInPixelVec_1));
    WorldPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(0, &PointsInWorldVec_0));
    WorldPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(1, &PointsInWorldVec_1));
    SetOrigin(0);
}

PoseSolver::PoseSolver(std::vector<cv::Point3f> &ptInWorld_0, std::vector<cv::Point3f> &ptInPixel_1, Config &config) : PointsInPixelVec_0(ptInWorld_0),PointsInPixelVec_1(ptInPixel_1), setting(config)
{
    SetOrigin(1);
}


PoseSolver::~PoseSolver()
{
}

void PoseSolver::SetOrigin(const int &flag){
    cv::Mat cvK = setting.ip.K;
    // cv::cv2eigen(cvK,eigenK);
    eigenK <<   cvK.at<float>(0,0), 0 , cvK.at<float>(0,2), 0,
                cvK.at<float>(1,1), cvK.at<float>(1,2),
                0, 0, 1;
    if (flag==0){
        Pose_0 = Eigen::Matrix4f::Identity();
        CamOrientation_0 << 0, 0, 1;
        Unproject(0,Pose_0);
    }else{
        Pose_0 = Eigen::Matrix4f::Identity();
        CamOrientation_0 << 0, 0, 1;
    }

}


void PoseSolver::Unproject(const int &PoseId, const Eigen::Matrix4f &T)
{
    for (size_t i(0); i< PixelPointsDict[PoseId]->size(); ++i){
        float u = PixelPointsDict[PoseId]->at(i).x;
        float v = PixelPointsDict[PoseId]->at(i).y;
        float z = PixelPointsDict[PoseId]->at(i).z;
        if (z <= 0) continue;
        Eigen::Vector3f PixelPoint(u, v, 1);
        // 像素点投影到相机坐标系下
        Eigen::Vector3f PointInCam = z*(eigenK.inverse()*PixelPoint);
        // std::cout << "R is " << std::endl << T.block(0,0,3,3).transpose() << std::endl;
        // std::cout << "t is " << std::endl << -T.block(0,0,3,3).transpose()*T.block<3,1>(0,3) << std::endl;
        // 这里的T表示 Tcw， 即从世界到相机
        Eigen::Vector3f PointInWorld = T.block(0,0,3,3).transpose() * ( PointInCam
            - T.block(0,0,3,3).transpose() * T.block<3,1>(0,3) );
        WorldPointsDict[PoseId]->emplace_back(PointInWorld(0), PointInWorld(1), PointInWorld(2));
    }
    // // CHECK POINT
    // std::cout << "CHECK POINT " << std::endl;
    // for (size_t i=0; i<WorldPointsDict[PoseId]->size(); ++i){
    //     std::cout << WorldPointsDict[PoseId]->at(i).x << "\t\t" <<
    //         WorldPointsDict[PoseId]->at(i).y << "\t\t" <<
    //         WorldPointsDict[PoseId]->at(i).z << std::endl;
    // }
}

void PoseSolver::ComputePnP()
{

    // std::vector<int> ls={10,20,30,40};
    // KneipPnP(ls,PointsInWorldVec_0,PointsInPixelVec_1);

    Optimization opts(setting);
    std::vector<int> Interior;
    Interior.resize(PointsInWorldVec_0.size());
    opts.Kneip_Ransac(PointsInWorldVec_0, PointsInPixelVec_1, Interior, T12_ransac);


    // CHECK POINT BEGIN
    CHECK_INFO("Before: ");
    CHECK_INFO_2("Size: ",PointsInWorldVec_0.size());
    double error = Average_ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, T12_ransac);
    CHECK_INFO_2("average error: ",error);

    std::vector< cv::Point3f > tmp_PW0, tmp_PP1;
    for (size_t i(0); i<Interior.size(); ++i){
        if(Interior[i]==1){
            tmp_PW0.emplace_back(PointsInWorldVec_0[i]);
            tmp_PP1.emplace_back(PointsInPixelVec_1[i]);
        }
    }

    // CheckReProjrctError();

    CHECK_INFO("After: ");
    PointsInWorldVec_0 = tmp_PW0;
    PointsInPixelVec_1 = tmp_PP1;
    CHECK_INFO_2("Size: ",PointsInWorldVec_0.size());
    double Interior_Error = Average_ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, T12_ransac);
    CHECK_INFO_2("average error: ",Interior_Error);

    CHECK_INFO("Optimization: ");
    opts.BA_OptimizePose(PointsInWorldVec_0, PointsInPixelVec_1, T12_ransac);
    double Optimization_Error = Average_ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, T12_ransac);
    // CHECK_INFO_2("Optimization error: ", Optimization_Error);
    std::cout << "[INFO] Optimization Error: " << Optimization_Error << std::endl;
}

void PoseSolver::EPnP_OpenCV(std::vector<int> &idVec, std::vector<cv::Point3f> &PointsInWorldVec_0, std::vector<cv::Point3f> &PointsInPixelVec_1)
{
    std::vector<cv::Point2f> pts_2d;
    std::vector<cv::Point3f> pts_3d(PointsInWorldVec_0);
    for (auto &pt : PointsInPixelVec_1)
        pts_2d.emplace_back(pt.x, pt.y);
    cv::Mat r,t,R;
    cv::solvePnP(pts_3d, pts_2d,setting.ip.K, cv::Mat(),r,t,false,cv::SOLVEPNP_EPNP);
    cv::Rodrigues(r,R);
    R = R.t();
    t = -R * t;
    T12(0, 0) = R.at<float>(0, 0);
    T12(0, 1) = R.at<float>(0, 1);
    T12(0, 2) = R.at<float>(0, 2);
    T12(1, 0) = R.at<float>(1, 0);
    T12(1, 1) = R.at<float>(1, 1);
    T12(1, 2) = R.at<float>(1, 2);
    T12(2, 0) = R.at<float>(2, 0);
    T12(2, 1) = R.at<float>(2, 1);
    T12(2, 2) = R.at<float>(2, 2);
    T12(0, 3) = t.at<float>(0, 0);
    T12(1, 3) = t.at<float>(1, 0);
    T12(2, 3) = t.at<float>(2, 0);
    T12(3, 0) = 0.0;
    T12(3, 1) = 0.0;
    T12(3, 2) = 0.0;
    T12(3, 3) = 1.0;
}


void PoseSolver::YuPnP(std::vector<int> &idVec, std::vector<cv::Point3f> &PointsInWorldVec_0,std::vector<cv::Point3f> &PointsInPixelVec_1)
{

    const int a = idVec[0];
    const int b = idVec[1];
    const int c = idVec[2];
    const int d = idVec[3];

    Eigen::Vector3f nx, ny, nz, P1, P2, P3;

    P1 << PointsInWorldVec_0[a].x, PointsInWorldVec_0[a].y, PointsInWorldVec_0[a].z;
    P2 << PointsInWorldVec_0[b].x, PointsInWorldVec_0[b].y, PointsInWorldVec_0[b].z;
    P3 << PointsInWorldVec_0[c].x, PointsInWorldVec_0[c].y, PointsInWorldVec_0[c].z;

    double x1, x2, x3, y1, y2, y3;
    Eigen::Vector3f p1,p2,p3;
    p1 << PointsInPixelVec_1[a].x, PointsInPixelVec_1[a].y, 1;
    p2 << PointsInPixelVec_1[b].x, PointsInPixelVec_1[b].y, 1;
    p3 << PointsInPixelVec_1[c].x, PointsInPixelVec_1[c].y, 1;
    p1 = eigenK.inverse() * p1;
    p2 = eigenK.inverse() * p2;
    p3 = eigenK.inverse() * p3;

    x1 = p1[0];
    y1 = p1[1];
    x2 = p2[0];
    y2 = p2[1];
    x3 = p3[0];
    y3 = p3[1];

    Eigen::Vector3f P12(P2-P1);
    Eigen::Vector3f P13(P3-P1);

    nx = P12.normalized();
    nz = nx.cross(P13).normalized();
    ny = nz.cross(nx);

    Eigen::Matrix3f N;
    N.row(0) = nx.transpose();
    N.row(1) = ny.transpose();
    N.row(2) = nz.transpose();

    Eigen::Vector3f P1_, P2_, P3_;
    P1_ << 0, 0, 0;
    P2_ = N * P12;
    P3_ = N * P13;
    double X2 = P2_(0);
    double X3 = P3_(0);
    double Y3 = P3_(1);

    // Eigen::Vector3f w1, w2, w3;
    // w1 << x2, (-x2*X3+x3*X3)*1.0/Y3, y2, (-y2*X3+y3*X3)*1.0/Y3, 0, 0, 1, 0, 0;
    // w2 << 0, x3, 0, y3, 0, 0, 0, 1, 0;
    // w3 << (x2-x1)/X2, ((x3-x1)*X2+(x1-x2)*X3)*1.0/(X2*Y3), (y2-y1)/X2,
    //         ((y3-y1)*X2+(y1-y2)*X3)*1.0/(X2*Y3), x1, y1, 0, 0, 1;

    double w11 = x2;
    double w21 = (-x2*X3+x3*X3)*1.0/Y3;
    double w31 = y2;
    double w41 = (-y2*X3+y3*X3)*1.0/Y3;
    // double w51 = 0;
    // double w61 = 0;
    // double w71 = 1;
    // double w81 = 0;
    // double w91 = 0;

    double w12 = 0;
    double w22 = x3;
    double w32 = 0;
    double w42 = y3;
    // double w52 = 0;
    // double w62 = 0;
    // double w72 = 0;
    // double w82 = 1;
    // double w92 = 0;

    double w13 = (x2-x1)/X2;
    double w23 = ((x3-x1)*X2+(x1-x2)*X3)*1.0/(X2*Y3);
    double w33 = (y2-y1)/X2;
    double w43 = ((y3-y1)*X2+(y1-y2)*X3)*1.0/(X2*Y3);
    // double w53 = x1;
    // double w63 = y1;
    // double w73 = 0;
    // double w83 = 0;
    // double w93 = 1;

    double f0 = w11*w21 + w31*w41;
    double f1 = w11*w22 + w31*w42 + 1;
    double f2 = w11*w23 + w21*w13 + w41*w33 + w31*w43;
    double f3 = w22*w13 + w42*w33;
    double f4 = w13*w23 + w33*w43;

    double g0 = w21*w21 + w41*w41 - w11*w11 - w31*w31 - 1;
    double g1 = 2*w21*w22 + 2*w41*w42;
    double g2 = w22*w22 + w42*w42 + 1;
    double g3 = 2*w21*w23 + 2*w41*w43 - 2*w11*w13 - 2*w31*w33;
    double g4 = 2*w22*w23 + 2*w42*w43;
    double g5 = w23*w23 + w43*w43 - w13*w13 - w33*w33;

    double h0 = g2*f2*f2 + g0*f1*f1 - g1*f0*f1;
    double h1 = -g4*f0*f1 + 2*g2*f0*f2 - g1*f0*f3 + g3*f1*f1 - g1*f1*f2 + 2*g0*f1*f3;
    double h2 = -g4*f0*f3 + 2*g2*f0*f4 + g5*f1*f1 - g4*f1*f2 + 2*g3*f1*f3 - g1*f1*f4 + g2*f2*f2 -g1*f2*f3 + g0*f3*f3;
    double h3 = 2*g5*f1*f3 - g4*f1*f4 - g4*f2*f3 + 2*g2*f2*f4 + g3*f3*f3 - g1*f3*f4;
    double h4 = g5*f3*f3 - g4*f3*f4 + g2*f4*f4;

    Eigen::Matrix<float,5,1> coff;
    std::vector<double> real_roots;

    coff << h0, h1, h2, h3, h4;
    solve_quartic_roots(coff,real_roots);
    assert(real_roots.size()==4);

    float minError = 9999;
    for(size_t i=0; i<4; ++i){
        double a1 = real_roots[i];
        double a2 = -(f0*a1*a1 + f2*a1 + f4)/(f1*a1 + f3);

        double s1 = a1*w11 + a2*w12 + w13;
        double s2 = a1*w21 + a2*w22 + w23;
        double s3 = a1*w31 + a2*w32 + w33;
        double s4 = a1*w41 + a2*w42 + w43;

        double tz = sqrt(2.0/(s1*s1 + s2*s2 + s3*s3 + s4*s4 + a1*a1 + a2*a2));
        double tx = x1*tz;
        double ty = y1*tz;
        Eigen::Vector3f t_co;
        t_co << tx, ty, tz;

        double r1 = s1*tz;
        double r2 = s2*tz;
        double r4 = s3*tz;
        double r5 = s4*tz;
        double r7 = a1*tz;
        double r8 = a2*tz;

        Eigen::Vector3f r_1, r_2, r_3;
        r_1 << r1, r4, r7;
        r_2 << r2, r5, r8;
        r_3 = r_1.cross(r_2);

        Eigen::Matrix3f R_co;
        R_co.col(0) = r_1;
        R_co.col(1) = r_2;
        R_co.col(2) = r_3;

        Eigen::Matrix3f R = R_co * N;
        Eigen::Vector3f t = t_co - R*P1;


        Eigen::Matrix4f T12_ = Eigen::Matrix4f::Zero();
        T12_.block(0,0,3,3) = R;
        T12_.block(0,3,3,1) = t;
        double error = ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, d, T12_);
        if (minError > error){
            minError = error;
            T12 = T12_;
        }


    }
    // std::cout << "[TEST] YuPnP MIN ERROR IS: " << minError << std::endl;
    // std::cout << "YuPnP T12\n " << T12 << std::endl;

}



void
PoseSolver::KneipPnP(std::vector<int> &idVec, std::vector<cv::Point3f> &PointsInWorldVec_0,std::vector<cv::Point3f> &PointsInPixelVec_1)
{
    const int a = idVec[0];
    const int b = idVec[1];
    const int c = idVec[2];
    const int d = idVec[3];
    Eigen::Vector3f f1, f2, f3, p1, p2, p3;
    Eigen::Vector3f tx, ty, tz;
    p1 << PointsInPixelVec_1[a].x, PointsInPixelVec_1[a].y, 1;
    p2 << PointsInPixelVec_1[b].x, PointsInPixelVec_1[b].y, 1;
    p3 << PointsInPixelVec_1[c].x, PointsInPixelVec_1[c].y, 1;

    double const colinear_threshold = 1e-10;
    if (((p2 - p1).cross(p3 - p1)).squaredNorm() < colinear_threshold)
        return;

    f1 << eigenK.inverse() * p1;
    f2 << eigenK.inverse() * p2;
    f3 << eigenK.inverse() * p3;
    f1 = f1.normalized();
    f2 = f2.normalized();
    f3 = f3.normalized();

    tx = f1;
    tz = (f1.cross(f2)).normalized();
    ty = tz.cross(tx);

    Eigen::Matrix3f T;
    T.row(0) = tx.transpose();
    T.row(1) = ty.transpose();
    T.row(2) = tz.transpose();
    f3 = T * f3;

    if ( f3(2)> 0.0){
        std::swap(f1, f2);
        std::swap(p1, p2);

        tx = f1;
        tz = (f1.cross(f2)).normalized();
        ty = tz.cross(tx);
        T.row(0) = tx.transpose();
        T.row(1) = ty.transpose();
        T.row(2) = tz.transpose();
        f3 = T * f3;
    }
    Eigen::Vector3f nx, ny, nz, P1, P2, P3;

    P1 << PointsInWorldVec_0[a].x, PointsInWorldVec_0[a].y, PointsInWorldVec_0[a].z;
    P2 << PointsInWorldVec_0[b].x, PointsInWorldVec_0[b].y, PointsInWorldVec_0[b].z;
    P3 << PointsInWorldVec_0[c].x, PointsInWorldVec_0[c].y, PointsInWorldVec_0[c].z;

    Eigen::Vector3f P12(P2-P1);
    Eigen::Vector3f P13(P3-P1);

    nx = P12.normalized();
    nz = nx.cross(P13).normalized();
    ny = nz.cross(nx);

    Eigen::Matrix3f N;
    N.row(0) = nx.transpose();
    N.row(1) = ny.transpose();
    N.row(2) = nz.transpose();
    P3 = N * (P13);

    double d12 = P12.norm();
    double cos_beta = f1.dot(f2);
    double bais = sqrt(1.0/( 1.0 - pow(cos_beta,2) )-1);

    if (cos_beta < 0.0)
        bais = -bais;

    double pha_1 = f3(0)/f3(2);
    double pha_2 = f3(1)/f3(2);

    Eigen::Matrix<float,5,1> coff;
    std::vector<double> real_roots;

    coff(4) =-2.0*pha_1*pha_2*P3(0)*pow(P3(1),2)*d12*bais
            + pow(pha_2*P3(1)*d12,2)
            + 2.0*pow(P3(0),3)*d12 - pow(P3(0)*d12,2)
            + pow(pha_2*P3(0)*P3(1),2) -pow(P3(0),4)
            - 2.0*pow(pha_2*P3(1),2)*P3(0)*d12
            + pow(pha_1*P3(0)*P3(1),2)
            + pow(pha_2*P3(1)*bais*d12,2);

    coff(3) = 2.0*pow(P3(0),2)*P3(1)*d12*bais
            + 2.0*pha_1*pha_2*d12*pow(P3(1),3)
            - 2.0*pow(pha_2,2)*pow(P3(1),3)*d12*bais
            - 2.0*P3(0)*P3(1)*bais*pow(d12,2);

    coff(2) = - pow(pha_2*P3(0)*P3(1),2)
            - pow(pha_2*P3(1)*d12*bais,2)
            - pow(pha_2*P3(1)*d12,2)
            + pow(P3(1),4)*(pow(pha_1,2)+pow(pha_2,2))
            + 2.0*P3(0)*d12*pow(P3(1),2)
            + 2.0*pha_1*pha_2*P3(0)*d12*bais*pow(P3(1),2)
            - pow(pha_1*P3(0)*P3(1),2)
            + 2.0*pow(pha_2*P3(1),2)*P3(0)*d12
            - pow(P3(1)*d12*bais,2) -2.0*pow(P3(0)*P3(1),2);

    coff(1) = 2.0 *pow(P3(1),3)*d12*bais
            + 2.0*pow(pha_2,2)*pow(P3(1),3)*d12*bais
            - 2.0*pha_1*pha_2*d12*pow(P3(1),3);

    coff(0) = -(pow(pha_2,2) + pow(pha_1,2) + 1)*pow(P3(1),4);

    solve_quartic_roots(coff,real_roots);
    assert(real_roots.size()==4);



    double minError = 9999;
    for (size_t i=0; i<4; ++i){
        double a1 = real_roots[i];
        double cot_alpha = (pha_1/pha_2*P3(0)+real_roots[i]*P3(1)-d12*bais)
            /(pha_1/pha_2*real_roots[i]*P3(1)- P3(0)+d12);

        // \theta \in [0,\pi];
        double cos_theta = real_roots[i];
        double sin_theta = sqrt(1-pow(cos_theta,2));
        // \alpha \in [0,\pi];
        double sin_alpha = sqrt(1.0/(pow(cot_alpha,2)+1));
        double cos_alpha = sqrt(1.0-(pow(sin_alpha,2)));
        if (cot_alpha < 0.0)
            cos_alpha = -cos_alpha;

        Eigen::Matrix3f R,Q;
        Eigen::Vector3f C_yita;
        C_yita(0) = d12*cos_alpha*(sin_alpha*bais+cos_alpha);
        C_yita(1) = d12*sin_alpha*cos_theta*(sin_alpha*bais+cos_alpha);
        C_yita(2) = d12*sin_alpha*sin_theta*(sin_alpha*bais+cos_alpha);

        Q << -cos_alpha, -sin_alpha*cos_theta, -sin_alpha*sin_theta,
             sin_alpha,  -cos_alpha*cos_theta, -cos_alpha*sin_theta,
             0.0,          -sin_theta,           cos_theta;

        CamOrientation_1 = P1 + N.transpose()*C_yita;
        R = N.transpose()*(Q.transpose()*T);
        R.transposeInPlace();
        Eigen::Vector3f t = -R*CamOrientation_1;


        Eigen::Matrix4f T12_;
        T12_ << R(0,0), R(0,1), R(0,2), t(0),
                R(1,0), R(1,1), R(1,2), t(1),
                R(2,0), R(2,1), R(2,2), t(2),
                0     , 0     , 0     , 1   ;
        double error = ReProjectError(PointsInWorldVec_0,PointsInPixelVec_1, d, T12_);
        if (minError > error){
            minError = error;
            T12 = T12_;
        }

    }

    // CHECK POINT
    // std::cout << "[TEST] KneipPnP MIN ERROR IS: " << minError << std::endl;
    // std::cout << "KneipPnP T12\n" << T12 << std::endl;
}


double PoseSolver::Average_ReProjectError(std::vector<cv::Point3f>&PointsInWorldVec_0
    ,std::vector<cv::Point3f>&PointsInPixelVec_1, Eigen::Matrix4f &T)
{
    assert( PointsInWorldVec_0.size() == PointsInPixelVec_1.size() );
    assert(!PointsInWorldVec_0.empty());
    double total_error = 0;
    for (size_t i(0); i<PointsInWorldVec_0.size();++i){
        double error = ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, i, T);
        total_error += error;
        // CHECK_INFO_2("error : ", error);
    }

    return total_error/PointsInWorldVec_0.size();
}


double PoseSolver::ReProjectError(std::vector<cv::Point3f>&PointsInWorldVec_0
    ,std::vector<cv::Point3f>&PointsInPixelVec_1,const int &i, Eigen::Matrix4f &T)
{
    Eigen::Vector3f u4, u4_;
    u4 << PointsInPixelVec_1[i].x, PointsInPixelVec_1[i].y, 1;
    Eigen::Vector4f P4;
    P4 << PointsInWorldVec_0[i].x, PointsInWorldVec_0[i].y, PointsInWorldVec_0[i].z, 1;
    Eigen::Vector4f CamCoord = T * P4;
    Project(CamCoord, u4_);
    double error = sqrt(pow(u4(0)-u4_(0),2)+pow(u4(1)-u4_(1),2));
    // std::cout << "[TEST] ERROR IS :" << error << std::endl;
    return error;
}

void PoseSolver::Project(Eigen::Vector4f &CamCoord, Eigen::Vector3f & PixelCoord){
    PixelCoord(0) = setting.ip.fx * CamCoord(0)/CamCoord(2) + setting.ip.cx;
    PixelCoord(1) = setting.ip.fy * CamCoord(1)/CamCoord(2) + setting.ip.cy;
    PixelCoord(2) = 1;
}


void PoseSolver::solve_quartic_roots(Eigen::Matrix<float,5,1> const& factors, std::vector<double> &real_roots)
{
    double const A = factors(0);
    double const B = factors(1);
    double const C = factors(2);
    double const D = factors(3);
    double const E = factors(4);

    double const A2 = A * A;
    double const B2 = B * B;
    double const A3 = A2 * A;
    double const B3 = B2 * B;
    double const A4 = A3 * A;
    double const B4 = B3 * B;

    double const alpha = -3.0 * B2 / (8.0 * A2) + C / A;
    double const beta = B3 / (8.0 * A3)- B * C / (2.0 * A2) + D / A;
    double const gamma = -3.0 * B4 / (256.0 * A4) + B2 * C / (16.0 * A3) - B * D / (4.0 * A2) + E / A;

    double const alpha2 = alpha * alpha;
    double const alpha3 = alpha2 * alpha;
    double const beta2 = beta * beta;

    std::complex<double> P(-alpha2 / 12.0 - gamma, 0.0);
    std::complex<double> Q(-alpha3 / 108.0 + alpha * gamma / 3.0 - beta2 / 8.0, 0.0);
    std::complex<double> R = -Q / 2.0 + std::sqrt(Q * Q / 4.0 + P * P * P / 27.0);

    std::complex<double> U = std::pow(R, 1.0 / 3.0);
    std::complex<double> y = (U.real() == 0.0)
                             ? -5.0 * alpha / 6.0 - std::pow(Q, 1.0 / 3.0)
                             : -5.0 * alpha / 6.0 - P / (3.0 * U) + U;

    std::complex<double> w = std::sqrt(alpha + 2.0 * y);
    std::complex<double> part1 = -B / (4.0 * A);
    std::complex<double> part2 = 3.0 * alpha + 2.0 * y;
    std::complex<double> part3 = 2.0 * beta / w;

    std::complex<double> complex_roots[4];
    complex_roots[0] = part1 + 0.5 * (w + std::sqrt(-(part2 + part3)));
    complex_roots[1] = part1 + 0.5 * (w - std::sqrt(-(part2 + part3)));
    complex_roots[2] = part1 + 0.5 * (-w + std::sqrt(-(part2 - part3)));
    complex_roots[3] = part1 + 0.5 * (-w - std::sqrt(-(part2 - part3)));

    for (int i = 0; i < 4; ++i)
        real_roots.push_back(complex_roots[i].real());
}

void PoseSolver::CheckReProjrctError()
{
    // 使用检查机制会破坏原有的数据，请仅在检查时才能使用
    PointsInWorldVec_0.clear();
    PointsInWorldVec_0.resize(4);
    PointsInPixelVec_1.clear();
    PointsInPixelVec_1.resize(4);
    cv::Point3f p1(-2.57094, -0.217018, 6.05338);
    cv::Point3f p2(-0.803123, 0.251818, 6.98383);
    cv::Point3f p3(2.05584, -0.607918, 7.52573);
    cv::Point3f p4(-0.62611418962478638, -0.80525958538055419, 6.7783102989196777);
    PointsInWorldVec_0[0] = p1;
    PointsInWorldVec_0[1] = p2;
    PointsInWorldVec_0[2] = p3;
    PointsInWorldVec_0[3] = p4;
    cv::Point3f uv1(-0.441758,-0.185523,1);
    cv::Point3f uv2(-0.135753,-0.0920593,1);
    cv::Point3f uv3(0.243795,-0.192743,1);
    cv::Point3f uv4(-0.11282696574926376,-0.24667978286743164,1);
    PointsInPixelVec_1[0] = uv1;
    PointsInPixelVec_1[1] = uv2;
    PointsInPixelVec_1[2] = uv3;
    PointsInPixelVec_1[3] = uv4;
    setting.ip.fx = 0.972222;
    setting.ip.fy = 0.972222;
    setting.ip.cx = 0.0;
    setting.ip.cy = 0.0;
    setting.ip.K.at<float>(0, 0) = setting.ip.fx;
    setting.ip.K.at<float>(1, 1) = setting.ip.fy;
    setting.ip.K.at<float>(0, 2) = setting.ip.cx;
    setting.ip.K.at<float>(1, 2) = setting.ip.cy;
    eigenK << setting.ip.fx, 0, setting.ip.cx, 0, setting.ip.fy, setting.ip.cy, 0, 0, 1;
    std::vector<int> ls={0,1,2,3};


    std::cout << "YuPnP : " << std::endl;

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    YuPnP(ls,PointsInWorldVec_0,PointsInPixelVec_1);
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    double t45 = std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t4).count();

    double error3 = Average_ReProjectError(PointsInWorldVec_0,PointsInPixelVec_1, T12);
    std::cout << "[INFO] ERROR IS: " << error3 << std::endl;
    std::cout << "YuPnP耗时： " << t45 << std::endl;

    if (error3 > 0.01)
        std::cerr << "[ERRO] PNP ERROR!" << std::endl;
    else
        std::cerr << "[INFO] PNP OK." << std::endl;

    std::cout << "==================================================" << std::endl;


    std::cout << "KneipPnP : " << std::endl;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    KneipPnP(ls,PointsInWorldVec_0,PointsInPixelVec_1);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double t01 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();

    double error = Average_ReProjectError(PointsInWorldVec_0,PointsInPixelVec_1, T12);
    std::cout << "[INFO] ERROR IS: " << error << std::endl;
    std::cout << "KneipPnP耗时： " << t01 << std::endl;

    if (error > 0.01)
        std::cerr << "[ERRO] PNP ERROR!" << std::endl;
    else
        std::cerr << "[INFO] PNP OK." << std::endl;

    std::cout << "==================================================" << std::endl;
    std::cout << "EPNP : " << std::endl;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    EPnP_OpenCV(ls, PointsInWorldVec_0, PointsInPixelVec_1);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double t23 = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();

    double error2 = Average_ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, T12);
    std::cout << "[INFO] ERROR IS: " << error2 << std::endl;
    std::cout << "EPnP耗时： " << t23 << std::endl;

    if (error2 > 0.01)
        std::cerr << "[ERRO] PNP ERROR!" << std::endl;
    else
        std::cerr << "[INFO] PNP OK." << std::endl;

    exit(0);

}

Eigen::Matrix4f PoseSolver::GetPose(){
    return T12_ransac;
}
