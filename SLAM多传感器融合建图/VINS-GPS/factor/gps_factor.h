//
// Created by grn on 11/30/18.
//

#ifndef VINS_GPS_FACTOR_H
#define VINS_GPS_FACTOR_H

#include <iostream>
#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>


struct gps_struct
{
    double gpspos[3]={0.0};
    double gpscov[3]={0.0};
    double time;
};

class GPSFactor : public ceres::SizedCostFunction< 3 , 7 >
{
  public:
    GPSFactor()=delete;
    GPSFactor(gps_struct _gpsdata):gpsdata(_gpsdata){}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d Pimu(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qimu(parameters[0][6], parameters[0][3], parameters[0][4],parameters[0][5]);
        Eigen::Vector3d Pgps(gpsdata.gpspos[0],gpsdata.gpspos[1],gpsdata.gpspos[2]);
        Eigen::Matrix<double, 3, 3> gpscovariance;
        gpscovariance.setZero();
        gpscovariance(0,0)=gpsdata.gpscov[0]*gpsdata.gpscov[0];gpscovariance(1,1)=gpsdata.gpscov[1]*gpsdata.gpscov[1];gpscovariance(2,2)=gpsdata.gpscov[2]*gpsdata.gpscov[2];
        Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);
        Eigen::Vector3d Lbgnss(GPS_L0,GPS_L1,GPS_L2);
        residual=Pgps-Qimu*Lbgnss-Pimu;
        Eigen::Matrix<double, 3, 3> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 3, 3>>(gpscovariance.inverse()).matrixL().transpose();
        residual = sqrt_info * residual;
        // cout<<sqrt_info<<endl;
        if(jacobians)
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
            jacobian.setZero();
            jacobian.block<3, 3>(0, 0)=-1.0*Eigen::Matrix3d::Identity();
            jacobian.block<3, 3>(0, 3)=-1.0*Utility::skewSymmetric(Qimu*Lbgnss);
            jacobian = sqrt_info * jacobian;
        }

        return true;
    }

    gps_struct gpsdata;

};


class GPSCost {

public:
    GPSCost(gps_struct _gpsdata):gpsdata(_gpsdata){}
    template <typename T>
    bool operator()(const T* const par , T* residuals) const {

        Matrix<T,3,1>  Pimu(par[0],par[1],par[2]);
        Quaternion<T>  Qimu(par[6],par[3],par[4],par[5]);
        Matrix<T,3,1>  Pgps(T(gpsdata.gpspos[0]),T(gpsdata.gpspos[1]),T(gpsdata.gpspos[2]));
        Matrix<T,3,1>  Lbgnss;
        Lbgnss.setZero();
        Lbgnss[0]=T(GPS_L0);
        Lbgnss[1]=T(GPS_L1);
        Lbgnss[2]=T(GPS_L2);

        Matrix<T,3,1> residual=Pgps-Qimu*Lbgnss-Pimu;

        residuals[0]=residual[0]*T(1.0/gpsdata.gpscov[0]);
        residuals[1]=residual[1]*T(1.0/gpsdata.gpscov[1]);
        residuals[2]=residual[2]*T(1.0/gpsdata.gpscov[2]);
        
        return true;
    }

    static ceres::CostFunction* Create(const gps_struct gps_data){
        return (new ceres::AutoDiffCostFunction<GPSCost,3,7>(
                new GPSCost(gps_data)));
    }

private:
    gps_struct gpsdata;
};



#endif //VINS_GPS_FACTOR_H
