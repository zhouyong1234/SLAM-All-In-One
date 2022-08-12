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


int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");

    MSCKF msckf;

    Frame lastFrame;
    int count = 0;

    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;

    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);
        //cout << "image_path = " << imagePath << endl;

        msckf.imageComing(image,iter->time_stamp);
        msckf.ConstructFrame();

        //msckf.ConstructFrame(image,iter->time_stamp);
        cout <<BOLDRED << "Frame id = " << msckf.frame.mnId << endl;

        cv::Mat imFeature = showFeatures(msckf.frame.mImageGray, msckf.frame.mvKeysUn);
        cv::imshow("features", imFeature);

        if(count > 0)
        {
            ORBmatcher orbMatcher(0.7);
            orbMatcher.MatcheTwoFrames(msckf.frame, lastFrame, false);
            cout << "matches = " << msckf.frame.matchesId.size() << endl;
            cv::Mat imMatch = DrawFrameMatch(msckf.frame, lastFrame);
            cv::imshow("matches", imMatch);
            cv::waitKey(0);

        }
        count++;
        if (count == 1)
            lastFrame = Frame(msckf.frame);

    }

    return 0;
}
