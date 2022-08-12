#ifndef MSCKF_H
#define MSCKF_H
#include "common_include.h"
#include "converter.h"
#include "camera.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "frame.h"
#include "types.h"
#include "types.h"
#include "triangulation.h"
#include "chi2table.h"
#include <ceres/ceres.h>


using namespace std;
using namespace cv;
using namespace Eigen;


namespace MSCKF_MINE
{

class MSCKF
{
public:
    MSCKF();
    ~MSCKF();

    /*constructer*/
    MSCKF(const VectorXd &state, const MatrixXd &P, const Vector3d &Acc, const Vector3d &Gyro, double &dt );

    /*EKF propagation variables member*/
    VectorXd mState;
    MatrixXd mCovariance;

    /*Camera*/
    Camera mCAMParams;

    /*IMU params*/
    IMU_PARAM mIMUParams;

    /*accelerometer and gyroscope measurement*/
    Vector3d mAccPrev;       /*[m s^-2]*/
    Vector3d mGyroPrev;      /*[rad s^-1]*/

    Vector3d mGravity;

    double mdt;

    Vector3d mbg;
    Vector3d mba;

    Matrix4d BigOmega( const Vector3d &w );
    Matrix3d skewMatrix(const Vector3d &v);

    void propagateIMU( Vector3d linear_acceleration, Vector3d angular_velocity);


    Matrix4d    calcOmegaMatrix(const Vector3d &w);
    Quaterniond calcDeltaQuaternion(const Vector3d &mGyroPrev, const Vector3d curr_w, double &dt);

    /*Augmentation cf. P.52*/
    void Augmentation();
    void Marginalizefilter();

    /*Feature part and update step*/

    /*ORB Feature Parts*/
    ORB_PARAM orbParam;
    ORBextractor* mpORBextractor;
    Mat mImage; /*it should be noted that the image was undistorted in function unDistorImage()*/
    double mTimeStamp;

    void imageComing(const cv::Mat &image, const double timestamp);

    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;

    void unDistortImage();
    void extractFeatures();

    Frame frame;
    Frame feedframe;
    bool mbReset;  /*only use in the first frame and the filter is reseted*/


    void ConstructFrame(const Mat &im, const double &timeStamp); /*for test*/
    void ConstructFrame(bool reset = false);

    void RunFeatureMatching();



    /* Residuals and Update step
     * Since we have extract features from the consecutive image
     * We have track this features and use the measurements to update
     * the msckf
     */
    /*Feature Mannager Parts*/
    vector<Eigen::MatrixXd> mvFeatures;
    vector<Eigen::Vector2i> mvFeaturesIdx;      /* i , imageNum*/
    vector<Eigen::MatrixXd> mvLostFeatures;
    vector<int>             mvLostFeatureCamIdx;

    void AugmentNewFeatures();
    void ManageOldFeatures();

    Vector3d TriangulationWorldPoint(vector<Vector2d> &z, VectorOfPose & poses);

    void CalcResidualsAndStackingIt();

    void CalcHxAndHf(Matrix4d &Tcw, Vector3d &pw, Matrix<double, 2,9> &Hbi,  Matrix<double, 2,3> &Hfi, Vector2d zij, Vector2d rij);
    void nullSpace(MatrixXd &H, MatrixXd &A);
    bool ChiSquareTest(MatrixXd &Hoi, MatrixXd &roi, MatrixXd &covariance);

    void MsckfUpdate(vector<MatrixXd> &vH, vector<MatrixXd> &vr);
    void QRdecomposition(MatrixXd H, MatrixXd r, MatrixXd &rq, MatrixXd &TH);
    void Update(MatrixXd &H,MatrixXd &rq, MatrixXd &TH);



};



}




#endif // MSCKF_H
