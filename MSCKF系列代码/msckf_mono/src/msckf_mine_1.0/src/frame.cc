#include "frame.h"
#include "opencv2/core.hpp"

namespace MSCKF_MINE
{
long unsigned int Frame::nNextId=0;
bool Frame::mbInitialFrame = true;
int    Frame::mnMaxCorners;
double Frame::mdQualityLevel;
int    Frame::mnMinDistance;
int    Frame::mnBlockSize;
bool   Frame::mbUseHarrisDetector;
double Frame::mdK;

Frame::Frame()
{

}

Frame::Frame(const Frame &frame)
    :mTimeStamp(frame.mTimeStamp),
      mCamera(frame.mCamera),
      mnId(frame.mnId),
      mImgGray(frame.mImgGray),
      mvCorners(frame.mvCorners),
      mvOldCorners(frame.mvOldCorners),
      mvNewCorners(frame.mvNewCorners)
{

}

Frame::Frame(const Mat &im, const double &timeStamp, int cornersNum, Mat mask = cv::Mat())
    :mTimeStamp(timeStamp)
{
    mnId = nNextId++;
    im.copyTo(mImgGray);

    /*Extract the Corners*/
    if(mbInitialFrame)
    {
        mbInitialFrame = false;
        mnMaxCorners        = Config::get<int>("Shi-Tomasi.maxCorners");
        mdQualityLevel      = Config::get<double>("Shi-Tomasi.qualityLevel");
        mnMinDistance       = Config::get<int>("Shi-Tomasi.minDistance");
        mnBlockSize         = Config::get<int>("Shi-Tomasi.blockSize");
        mbUseHarrisDetector = false;
        mdK                 = Config::get<double>("Shi-Tomasi.k");

        ExtractFeatures(mask,mnMaxCorners);
    }
    else
    {
        ExtractFeatures(mask,cornersNum);
    }

}

Frame::Frame(const Mat &im, const double &timeStamp, int cornersNum, Mat mask, vector<Point2f> oldCorners)
    :mTimeStamp(timeStamp),
      mvOldCorners(oldCorners)
{
    mnId = nNextId++;
    im.copyTo(mImgGray);
    /*Before we Extract the Corners, We need to undistort the image*/

    /*Extract the Corners*/
    if(mbInitialFrame)
    {
        mbInitialFrame = false;
        mnMaxCorners        = Config::get<int>("Shi-Tomasi.maxCorners");
        mdQualityLevel      = Config::get<double>("Shi-Tomasi.qualityLevel");
        mnMinDistance       = Config::get<int>("Shi-Tomasi.minDistance");
        mnBlockSize         = Config::get<int>("Shi-Tomasi.blockSize");
        mbUseHarrisDetector = false;
        mdK                 = Config::get<double>("Shi-Tomasi.k");

        ExtractFeatures(mask,mnMaxCorners);
    }
    else
    {
        ExtractFeatures(mask,cornersNum);
    }

}

void Frame::UnDistortImg()
{
    cv::Mat tmp = mImgGray.clone();
    cv::undistort(tmp, mImgGray, mCamera.getK(), mCamera.getD(), cv::Mat());

}

/*
 * output --> mvCorners
 */
void Frame::ExtractFeatures(Mat &mask, int cornersNum)
{
    mvCorners.clear();
    if(cornersNum > 0)
    {
        mvNewCorners.clear();
        goodFeaturesToTrack( mImgGray,
                             mvNewCorners,
                             cornersNum,
                             mdQualityLevel,
                             mnMinDistance,
                             mask,
                             mnBlockSize,
                             mbUseHarrisDetector,
                             mdK );

        cv::TermCriteria criteria = cv::TermCriteria(
                    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                    40,
                    0.01);

        cv::cornerSubPix(mImgGray, mvNewCorners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    }


    /*Contact the old and new features*/
    if(mvOldCorners.empty())
    {
        mvCorners = mvNewCorners;
    }
    else if(mvNewCorners.empty())
    {
        mvCorners = mvOldCorners;
    }
    else
    {
        mvCorners = mvOldCorners;
        mvCorners.insert(mvCorners.end(),mvNewCorners.begin(),mvNewCorners.end());
    }
}

/*
 * output is mask
 */
void Frame::GenerateMask(Mat &mask, vector<cv::Point2f> &corners)
{
    size_t r = 5;
    /*the mask must be CV_8UC1*/
    for(vector<Point2f>::iterator iter_corner = corners.begin(); iter_corner!=corners.end(); iter_corner++ )
    {
        Point2f &corner = *iter_corner;

        float top_left_x = corner.x - r;

        if(top_left_x > mask.cols)
            top_left_x = mask.cols;
        else if (top_left_x < 0)
            top_left_x = 0;

        float top_left_y = corner.y - r;

        if(top_left_y > mask.rows)
            top_left_y = mask.rows;
        else if (top_left_y < 0)
            top_left_y = 0;

        Point2f top_left = Point2f(top_left_x,top_left_y);

        float bottom_right_x = corner.x + r;

        if(bottom_right_x > mask.cols)
            bottom_right_x = mask.cols;
        else if (bottom_right_x < 0)
            bottom_right_x = 0;

        float bottom_right_y = corner.y + r;

        if(bottom_right_y > mask.rows)
            bottom_right_y = mask.rows;
        else if (bottom_right_y < 0)
            bottom_right_y = 0;

        Point2f bottom_right = Point2f(bottom_right_x,bottom_right_y);

        cv::rectangle(mask, top_left, bottom_right, Scalar(0), -1, 8);

    }

}




}
