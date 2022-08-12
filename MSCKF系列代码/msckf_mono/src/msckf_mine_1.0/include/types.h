#ifndef TYPES_H
#define TYPES_H
#include "common_include.h"
#include "config.h"

namespace MSCKF_MINE
{
class IMU
{
public:
    /*
         * timestamp [ns],
         * w_RS_S_x [rad s^-1],  w_RS_S_y [rad s^-1],   w_RS_S_z [rad s^-1],
         * a_RS_S_x [m s^-2],    a_RS_S_y [m s^-2],     a_RS_S_z [m s^-2]
         */
    double time_stamp;
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
};

class CAMERA
{
public:
    double time_stamp;
    string img_name;
    string img_name1;
};



class IMU_PARAM
{
public:
    IMU_PARAM()
    {
        g          = Config::get<double>("g");
        sigma_ac   = Config::get<double>("accelerometer_noise_density");    // [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
        sigma_gc   = Config::get<double>("gyroscope_noise_density");        // [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
        sigma_wac  = Config::get<double>("accelerometer_random_walk");                      // [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
        sigma_wgc  = Config::get<double>("gyroscope_random_walk");                     // [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
    }

    double g;
    double sigma_ac;
    double sigma_gc;
    double sigma_wac;
    double sigma_wgc;

};

class ORB_PARAM
{
public:
    ORB_PARAM()
    {
        nFeatures    = Config::get<int>("ORBextractor.nFeatures");
        scaleFactor  = Config::get<float>("ORBextractor.scaleFactor");
        nLevels      = Config::get<int>("ORBextractor.nLevels");
        iniThFAST    = Config::get<int>("ORBextractor.iniThFAST");
        minThFAST    = Config::get<int>("ORBextractor.minThFAST");

    }
    int nFeatures;
    float scaleFactor;
    int nLevels;
    int iniThFAST;
    int minThFAST;
};

struct Pose {
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::vector<Pose, Eigen::aligned_allocator<Pose> > VectorOfPose;


}

#endif // TYPES_H
