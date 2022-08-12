#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "msckf.h"
#include "data_reader.h"
#include "ORBextractor.h"
#include "frame.h"
#include "ORBmatcher.h"
#include <vector>

using namespace MSCKF_MINE;



cv::Mat showFeatures(const cv::Mat &mImage, const vector<KeyPoint> &mvKeys)
{
    cv::Mat imageWithFeatures = mImage.clone();
    cv::cvtColor(imageWithFeatures, imageWithFeatures, CV_GRAY2BGR);
    for(vector<KeyPoint>::const_iterator iter= mvKeys.begin(); iter!=mvKeys.end(); iter++ )
    {
        KeyPoint point = *iter;
        cv::circle(imageWithFeatures, Point(point.pt.x, point.pt.y), 3, Scalar(0,255,0), 1);
    }

    return imageWithFeatures.clone();

}

cv::Mat DrawFrameMatch(Frame &mCurrentFrame, Frame &mLastFrame)
{
    cv::Mat imMatch = Mat::zeros(mCurrentFrame.mImageGray.rows*2,mCurrentFrame.mImageGray.cols,CV_8UC3);
    vector<cv::KeyPoint> vCurrentKeys = mCurrentFrame.mvKeys;
    vector<cv::KeyPoint> vLastKeys    = mLastFrame.mvKeys;
    cv::Mat curr_Im, last_Im;



    mCurrentFrame.mImageGray.copyTo(curr_Im);

    mLastFrame.mImageGray.copyTo(last_Im);
    //    cout << "size = " << mLastFrame.mImageGray.rows << endl;

    //    cv::imshow("last", mLastFrame.mImageGray);
    //cv::waitKey(0);
    if(curr_Im.channels()<3) //this should be always true
    {
        cvtColor(curr_Im,curr_Im,CV_GRAY2BGR);

    }

    if(last_Im.channels()<3) //this should be always true
    {
        cvtColor(last_Im,last_Im,CV_GRAY2BGR);

    }

    /*matches*/
    map<int,int> matches = mCurrentFrame.matchesId;
    const float r = 5;

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt1,pt2,pt3,pt4;
        pt1.x=vCurrentKeys[curr_id].pt.x-r;
        pt1.y=vCurrentKeys[curr_id].pt.y-r;
        pt2.x=vCurrentKeys[curr_id].pt.x+r;
        pt2.y=vCurrentKeys[curr_id].pt.y+r;

        pt3.x=vLastKeys[last_id].pt.x-r;
        pt3.y=vLastKeys[last_id].pt.y-r;
        pt4.x=vLastKeys[last_id].pt.x+r;
        pt4.y=vLastKeys[last_id].pt.y+r;


        //cv::rectangle(curr_Im,pt1,pt2,cv::Scalar(255,0,0));
        cv::circle(curr_Im,vCurrentKeys[curr_id].pt,2,cv::Scalar(0,255,0),-1);

        //cv::rectangle(last_Im,pt3,pt4,cv::Scalar(0,255,0));
        cv::circle(last_Im,vLastKeys[last_id].pt,2,cv::Scalar(255,0,0),-1);


    }



    curr_Im.copyTo(imMatch(cv::Rect(0,0,curr_Im.cols,curr_Im.rows)));
    last_Im.copyTo(imMatch(cv::Rect(0,curr_Im.rows,curr_Im.cols,curr_Im.rows)));
    /*now we need to draw the lines*/

    for(map<int,int>::iterator matches_iter = matches.begin(); matches_iter!=matches.end();matches_iter++)
    {

        int curr_id = matches_iter->first;
        int last_id = matches_iter->second;
        /*draw the current frame, the first value*/
        cv::Point2f pt_curr,pt_last;

        pt_curr = vCurrentKeys[curr_id].pt;
        pt_last.x = vLastKeys[last_id].pt.x;
        pt_last.y = vLastKeys[last_id].pt.y + curr_Im.rows;


        cv::line(imMatch, pt_curr, pt_last, cv::Scalar(0,0,255),1,8);


    }
    return imMatch;
}


void showMsckfAllDate(MSCKF &msckf)
{
    cout << BOLDRED"<---Below are msckf data-->" << endl;
    cout << BOLDCYAN"--mState = \n" << BOLDGREEN << msckf.mState << WHITE <<endl;
//    cout << BOLDCYAN"--mCovariance = \n" << BOLDGREEN << msckf.mCovariance << WHITE <<endl;
//    cout << BOLDCYAN"--mAccPrev = \n" << BOLDGREEN << msckf.mAccPrev << WHITE <<endl;
//    cout << BOLDCYAN"--mGyroPrev = \n" << BOLDGREEN << msckf.mGyroPrev << WHITE <<endl;
//    cout << BOLDCYAN"--mGravity = \n" << BOLDGREEN << msckf.mGravity << WHITE <<endl;
//    cout << BOLDCYAN"--mdt = \n" << BOLDGREEN << msckf.mdt << WHITE <<endl;

}

int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");

    //    MSCKF msckf;

    Frame lastFrame;



    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;
    vector<IMU> vImuData = data.mvImuData;

    VectorXd X0 = VectorXd::Zero(16,1);
    MatrixXd P0 = MatrixXd::Zero(15,15);

    Quaterniond q0 = Quaterniond(0.534108, -0.153029,  -0.827383,  -0.082152);

    X0.segment<4>(0) = q0.coeffs();
    X0.segment<3>(4) = Vector3d(4.688319,  -1.786938,  0.783338);
    X0.segment<3>(10) = Config::get<double>("gyroscope_random_walk") * Vector3d(1,1,1);
    X0.segment<3>(13) = Config::get<double>("accelerometer_random_walk") * Vector3d(1,1,1);


    Vector3d Acc(vImuData[0].ax, vImuData[0].ay, vImuData[0].az);
    Vector3d Gyro(vImuData[0].wx, vImuData[0].wy, vImuData[0].wz);
    double lastImuTime, Imutime;
    Imutime = lastImuTime = vImuData[0].time_stamp;
    double dt = 0.0;
    int countImu = 1;

    MSCKF msckf(X0, P0, Acc, Gyro, dt);

    showMsckfAllDate(msckf);


    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);

        while (Imutime < iter->time_stamp)
        {
            /*which means the image haven't coming*/
            IMU imu = vImuData[countImu];
            Imutime = imu.time_stamp;
            countImu++;

            msckf.mdt = (Imutime - lastImuTime) / 1000000000;


            lastImuTime = Imutime;
            msckf.propagateIMU(Vector3d(imu.ax, imu.ay, imu.az), Vector3d(imu.wx,imu.wy,imu.wz));

//            showMsckfAllDate(msckf);

        }


        /*when an image is coming*/
        msckf.imageComing(image, iter->time_stamp);
        msckf.Augmentation();
        msckf.RunFeatureMatching();

        cout << BOLDCYAN"<-----Image " << msckf.frame.mnId << " is coming!----->" << BOLDWHITE <<  endl;

        showMsckfAllDate(msckf);
        int imageNum = (msckf.mState.size() - 16 ) / 10;
        cout << "image num = " << imageNum << endl;
        cout << "feedframe:" << msckf.feedframe.mnId << "--" << "currentFrame: " << msckf.frame.mnId << endl;

//        cin.get();
        if(msckf.frame.mnId >= 2 && !msckf.mbReset)
            msckf.CalcResidualsAndStackingIt();

    }

    return 0;
}
