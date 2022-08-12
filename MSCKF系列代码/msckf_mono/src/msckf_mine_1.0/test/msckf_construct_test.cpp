#include "common_include.h"
#include "camera.h"
#include "converter.h"
#include "config.h"
#include "msckf.h"

using namespace MSCKF_MINE;

int main(int argc, char *argv[])
{
    // imu state vector order:    quaternion, position, velocity, bg, ba
    Config::setParameterFile("../config/config.yaml");
    VectorXd state = VectorXd::Zero(16,1);
    MatrixXd P = MatrixXd::Zero(15,15);

    double dt = 0.0;

    Vector3d Gyro = Vector3d(-0.099134701513277898,0.14730578886832138,0.02722713633111154);
    Vector3d Acc  = Vector3d(8.1476917083333333,-0.37592158333333331,-2.4026292499999999);

    MSCKF msckf(state, P, Acc, Gyro, dt);

    cout << BOLDCYAN << "---Test the basic function of MSCKF---" << WHITE << endl;

    cout << "msckf variables:\n";
    cout << "--state = \n" << msckf.mState << endl;
    cout << "--covariance = \n" << msckf.mCovariance << endl;
    cout << "--AccMeasurements = \n" << msckf.mAccPrev << endl;
    cout << "--GyroMeasurements = \n" << msckf.mGyroPrev << endl;
    cout << "--CameraParams:\n";
    cout << "--sigma_img= " << msckf.mCAMParams.sigma_img << endl;
    cout << "--camera_intrinsic = \n" << msckf.mCAMParams.getK() << endl;
    cout << "--IMUParams:\n";
    cout << "--sigma_ac = " << msckf.mIMUParams.sigma_ac << endl;

    cout << "--test basic funtion" << endl;

    Vector3d w(1.1, 2.2, 3.3);
    Matrix3d W = msckf.skewMatrix(w);
    Matrix4d bigW = msckf.BigOmega(w);

    cout << "--w = \n" << w << endl;
    cout << "--W = \n" << W << endl;
    cout << "--bigW = \n" << bigW << endl;


    return 0;
}
