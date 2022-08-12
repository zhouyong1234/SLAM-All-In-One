#ifndef MSCKF_H
#define MSCKF_H
#include "common_include.h"
#include "converter.h"
#include "camera.h"
#include "types.h"
#include "triangulation.h"
#include "chi2table.h"
#include "frame.h"
#include "feature.h"
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

    int GetSlideWindowSize();

    /*Augmentation cf. P.52*/
    void Augmentation();
    void Marginalizefilter();

    /*Feature part and update step*/

    /*Feature Parts*/
    Mat mImage; /*it should be noted that the image was undistorted in function unDistorImage()*/
    double mTimeStamp;
    unsigned int mnFeatureId;

    void imageComing(const cv::Mat &image, const double timestamp);

    void unDistortImage();

    Frame mCurrFrame;
    Frame mLastFrame;
    VectorOfFeatures mvFeatureContainer;
    vector<Point2f>  mvCorners;             /*only keep the exist corners*/
    bool mbReset;                           /*only use in the first frame and the filter is reseted*/
    int mnMaxLifeTime;


    /*Tracking Part*/
    /* Residuals and Update step
     * Since we have extract features from the consecutive image
     * We have track this features and use the measurements to update
     * the msckf
     */
    void Tracking();
    void OpticalFlowTracking();
    void CornersToFeatures(Frame &frame);


    VectorOfFeatures mvFeaturesForUpdate;
    int mnSWFrameId; /*since we maintain a Slide Window, we should know the first frame id because we will erase some if the state vector too large*/


    Vector3d TriangulationWorldPoint(vector<Vector2d> &z, VectorOfPose & poses, ceres::Problem *problem);

    void CalcResidualsAndStackingIt();

    void CalcHxAndHf(Matrix4d &Tcw, Vector3d &pw, Matrix<double, 2,9> &Hbi,  Matrix<double, 2,3> &Hfi, Vector2d &zij, Vector2d &rij);
    void nullSpace(MatrixXd &H, MatrixXd &A);
    bool ChiSquareTest(MatrixXd &Hoi, MatrixXd &roi, MatrixXd &covariance);

    int mnUpdateCount;
    void MsckfUpdate(vector<MatrixXd> &vH, vector<VectorXd> &vr);
    void QRdecomposition(MatrixXd H, VectorXd r, VectorXd &rq, MatrixXd &TH);
    void Update(MatrixXd &H, VectorXd &rq, MatrixXd &TH);

    /*Show the state*/
    void ShowState(bool flag_R = false, bool flag_t = true, bool flag_v = false);

};



}




#endif // MSCKF_H
