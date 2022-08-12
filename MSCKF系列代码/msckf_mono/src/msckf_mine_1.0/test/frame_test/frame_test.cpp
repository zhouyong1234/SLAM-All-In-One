#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "msckf.h"
#include "data_reader.h"
#include "frame.h"
#include <vector>

using namespace MSCKF_MINE;

Mat ShowFeatures(Frame &frame);

int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");

    /*Initial the msckf*/
    // imu state vector order:    quaternion, position, velocity, bg, ba
    VectorXd X0 = VectorXd::Zero(16,1);
    MatrixXd P0 = MatrixXd::Zero(15,15);

    Quaterniond orient = Quaterniond( 0.534108, -0.153029,  -0.827383,  -0.082152);

    X0.segment<4>(0) = orient.coeffs(); // x y z w
    X0.segment<3>(4) = Vector3d(4.688319,  -1.786938,  0.783338);
    X0.segment<3>(10) = 1.9393e-05*Vector3d(1, 1, 1);
    X0.segment<3>(13) = 3.0000e-3*Vector3d(1, 1, 1);



    Vector3d B_wPrev = Vector3d(-0.099134701513277898,0.14730578886832138,0.02722713633111154);
    Vector3d B_aPrev = Vector3d(8.1476917083333333,-0.37592158333333331,-2.4026292499999999);
    double dt = 0.0;
    MSCKF msckf = MSCKF(X0, P0, B_aPrev, B_wPrev, dt);

    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;

    string imagePath = vCameraData[0].img_name;
    double timeStamp = vCameraData[0].time_stamp;
    Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);
    msckf.imageComing(image, timeStamp);


    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);

        msckf.imageComing(image,iter->time_stamp);

        cout <<BOLDRED << "frame id = " << msckf.mLastFrame.mnId << endl;

        cout << msckf.mvFeatureContainer.size() << endl;

//        /*the 15th feature information*/
//        cout << "Information of 15th feature" << endl;
//        cout << "Feature ID: " << msckf.mvFeatureContainer[14].mnId << endl;
//        cout <<BOLDGREEN << "Frame ID:"<< msckf.mvFeatureContainer[14].mnFrameId <<WHITE <<  endl;
//        Feature &feature = msckf.mvFeatureContainer[14];
//        for(int i = 0; i < feature.mvObservation.size(); i++)
//        {
//            cout << "The " << i << "th observation\n" <<feature.mvObservation[i] << endl;
//        }

        Mat imFeature = ShowFeatures(msckf.mLastFrame);

        /*information of msckf and frame*/

        cv::imshow("features", imFeature);
        cv::waitKey(0);

    }

    return 0;
}

Mat ShowFeatures(Frame &frame)
{
    vector<Point2f> &oldcorners = frame.mvOldCorners;
    vector<Point2f> &newcorners = frame.mvNewCorners;
    Mat image = frame.mImgGray;
    cvtColor(image,image,CV_GRAY2BGR);
    for(int i = 0; i < oldcorners.size(); i++)
    {
        cv::circle( image, oldcorners[i], 3, Scalar(0,0,255), -1, 8, 0 );
    }
    for(int i = 0; i < newcorners.size(); i++)
    {
        cv::circle( image, newcorners[i], 3, Scalar(0,255,0), -1, 8, 0 );
    }
    return image;
}
