#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "converter.h"
#include "types.h"

namespace MSCKF_MINE
{
class Frame
{
public:
    Frame();

    Frame(const Frame &frame);

    Frame(const Mat &im, const double &timeStamp, int cornersNum, Mat mask);
    Frame(const Mat &im, const double &timeStamp, int cornersNum, Mat mask, vector<Point2f> oldCorners);

    double mTimeStamp;

    Camera mCamera;

    unsigned int mnId;

    static long unsigned int nNextId;

    cv::Mat mImgGray;

    static bool mbInitialFrame;

    /*Features*/
    vector<Point2f> mvOldCorners;
    vector<Point2f> mvNewCorners;
    vector<Point2f> mvCorners;


    /*Below are some config params*/
    static int    mnMaxCorners;
    static double mdQualityLevel;
    static int    mnMinDistance;
    static int    mnBlockSize;
    static bool   mbUseHarrisDetector;
    static double mdK;

public:

    void ExtractFeatures(Mat &mask, int cornersNum);

    void UnDistortImg();

    void GenerateMask(Mat &mask, vector<Point2f> &corners);

};

}



#endif // FRAME_H
