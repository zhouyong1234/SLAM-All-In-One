#ifndef VINS_C00SYS_H
#define VINS_C00SYS_H

#include <iostream>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;



class myPoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        Eigen::Map<const Eigen::Vector3d> _p(x+1);
        Eigen::Map<const Eigen::Quaterniond> _q(x+4);
        
        Eigen::Map<const Eigen::Vector3d> dp(delta+1);

        Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 4));

        Eigen::Map<Eigen::Vector3d> p(x_plus_delta + 1);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 4);

        x_plus_delta[0]=x[0]+delta[0];
        p = _p + dp;
        q = (_q * dq).normalized();

        return true;
    };
    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 8, 7, Eigen::RowMajor>> j(jacobian);

        j.topRows<7>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    };
    virtual int GlobalSize() const { return 8; };
    virtual int LocalSize() const { return 7; };
};


class MyScalarCostFunctor {

public:
    MyScalarCostFunctor(Vector3d pw,Vector3d pc): pw_(pw), pc_(pc) {}
    template <typename T>
    bool operator()(const T* const par , T* residuals) const {

        Matrix<T, 3, 1>  pww,pcc;
        pww[0]=T(pw_[0]);
        pww[1]=T(pw_[1]);
        pww[2]=T(pw_[2]);
        pcc[0]=T(pc_[0]);
        pcc[1]=T(pc_[1]);
        pcc[2]=T(pc_[2]);
        T s=par[0];
        Matrix<T, 3, 1> tt(par[1],par[2],par[3]);
        Quaternion<T> qq(par[7],par[4],par[5],par[6]);
        Matrix<T, 3, 1> pre=pww-(s*(qq*pcc)+tt);
        residuals[0] = pre[0];
        residuals[1] = pre[1];
        residuals[2] = pre[2];
        return true;
    }

    static ceres::CostFunction* Create(const Vector3d pww, const Vector3d pcc){
        return (new ceres::AutoDiffCostFunction<MyScalarCostFunctor,3,8>(
                new MyScalarCostFunctor(pww,pcc)));
    }

private:
    Vector3d pw_;
    Vector3d pc_;
};


inline int getRTWC(const vector<Vector3d> &pws, const vector<Vector3d> &pcs, Matrix3d &RWC , Vector3d &TWC ,double &SWC ) {

    double par[8]={1,0,0,0,0,0,0,1};
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    ceres::LocalParameterization *local_parameterization = new myPoseLocalParameterization();
    problem.AddParameterBlock(par, 8, local_parameterization);

    for(size_t i=0;i<pws.size();i++)
    {
        ceres::CostFunction* cost_function=MyScalarCostFunctor::Create(pws[i],pcs[i]);
        problem.AddResidualBlock(cost_function, loss_function, par);
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 8;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations=100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Quaterniond q(par[7],par[4],par[5],par[6]);
    
    RWC=q.toRotationMatrix();
    TWC[0]=par[1];
    TWC[1]=par[2];
    TWC[2]=par[3];
    SWC=par[0];
    
    double sum=0;
    int num=pws.size();

    for(size_t i=0;i<pws.size();i++)
    {
        Vector3d pww=SWC*(RWC*pcs[i])+TWC;
        Vector3d dis=pws[i]-pww;
        cout<<"********"<<dis.transpose()<<endl;
        double dd=sqrt(dis[0]*dis[0]+dis[1]*dis[1]+dis[2]*dis[2]);
        sum+=dd;
    }

    if(sum/num > 0.05)
        return 0;

    return 1;
}



inline int getRTWC2(const vector<Vector3d> &pws, const vector<Vector3d> &pcs, Matrix3d &RWC , Vector3d &TWC ,double &SWC ) 
{
    Vector2d pw0(pws[0][0],pws[0][1]);
    Vector2d pw1(pws[1][0],pws[1][1]);
    Vector2d pc0(pcs[0][0],pcs[0][1]);
    Vector2d pc1(pcs[1][0],pcs[1][1]);
    Vector2d vw=pw1-pw0;
    Vector2d vc=pc1-pc0;
    double yaw1=atan2(vw[1],vw[0]);
    double yaw2=atan2(vc[1],vc[0]);
    double yaw=yaw1-yaw2;
    double s=vw.norm()/vc.norm();
    Matrix2d R;
    R<<cos(yaw),-sin(yaw),
       sin(yaw), cos(yaw);
    Vector2d Pc0=R*pc0*s;
    Vector2d Pc1=R*pc1*s;
    Vector2d a=pw1-Pc1;
    Vector2d b=pw0-Pc0;
    cout<<"*****"<<a.transpose()<<endl;
    cout<<"*****"<<b.transpose()<<endl;

    SWC=s;
    TWC[0]=(a[0]+b[0])/2.0;
    TWC[1]=(a[1]+b[1])/2.0;
    TWC[2]=0.0;
    RWC<< cos(yaw),-sin(yaw),0.0,
          sin(yaw), cos(yaw),0.0,
          0.0        , 0.0  ,1.0;

    double sum=0;
    int num=pws.size();
    double kkk=0.0;
    for(size_t i=0;i<pws.size();i++)
    {
        Vector3d pww=SWC*(RWC*pcs[i])+TWC;
        // cout<<"pcs********"<<pcs[i].transpose()<<endl;
        // cout<<"pww********"<<pww.transpose()<<endl;
        // cout<<"pws********"<<pws[i].transpose()<<endl;
        Vector3d dis=pws[i]-pww;
        kkk+=dis[2];
        cout<<"********"<<dis.transpose()<<endl;
        double dd=sqrt(dis[0]*dis[0]+dis[1]*dis[1]);
        sum+=dd;
    }

    TWC[2]=kkk/num;

    if(sum/num > 0.05)
        return 0;

    return 1;
    
}


#endif