#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "msckf.h"
#include "data_reader.h"
#include "frame.h"
#include <vector>

using namespace MSCKF_MINE;

cv::Mat ShowFeatures(Frame &frame);

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


    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;
    vector<IMU>    vImu = data.mvImuData;
    double imu_time = vImu[0].time_stamp;
    double pre_imu_time = imu_time;
    dt = (imu_time - pre_imu_time) / 1000000000.;
    cout << "In intial step, dt = " << dt << endl;
    MSCKF msckf = MSCKF(X0, P0, B_aPrev, B_wPrev, dt);
    msckf.ShowState();

    unsigned int imu_index = 0;
    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);
        double cam_time = iter->time_stamp;

        cout << BOLDYELLOW"---In IMU propagate Part---" << WHITE << endl;
        while (imu_time <= cam_time )
        {

            imu_index++;
            /*For this part, we only propagate the imu.*/
            imu_time = vImu[imu_index].time_stamp;

            cout << "Current IMU timestamp is: " << fixed << setprecision(0) <<  imu_time << endl;
//            cout << "Last    IMU timestamp is: " << fixed << setprecision(0) <<  pre_imu_time << endl;
            dt = (imu_time - pre_imu_time) / 1000000000.;
            cout << "       The delta time is: " << fixed << setprecision(8) << dt << endl;
            msckf.mdt = dt;

            /*Construct the acc and gyro data*/
            Vector3d acc  = Vector3d(vImu[imu_index].ax, vImu[imu_index].ay, vImu[imu_index].az);
            Vector3d gyro = Vector3d(vImu[imu_index].wx, vImu[imu_index].wy, vImu[imu_index].wz);

            msckf.propagateIMU(acc, gyro);
            msckf.ShowState();

            pre_imu_time = imu_time;

        }
        cout << BOLDYELLOW"---IMU propagate Part ended---" << WHITE << endl << endl << endl;

        cout << endl << BOLDRED"---Camera part -> Image is coming---" << WHITE << endl;

        msckf.imageComing(image,iter->time_stamp);

        cout <<BOLDRED << "frame id = " << msckf.mLastFrame.mnId << WHITE << endl;


        Mat imFeature = ShowFeatures(msckf.mLastFrame);

        /*information of msckf and frame*/

        cv::imshow("features", imFeature);
        cv::waitKey(0);

        cout << BOLDRED"---Camera part -> Image is done---" << WHITE << endl << endl << endl;

    }

    return 0;
}

cv::Mat ShowFeatures(Frame &frame)
{
    vector<Point2f> &oldcorners = frame.mvOldCorners;
    vector<Point2f> &newcorners = frame.mvNewCorners;
    cv::Mat image = frame.mImgGray.clone();
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
