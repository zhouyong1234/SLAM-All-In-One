#include <iostream>
#include <opencv2/opencv.hpp>
#include "data_reader.h"
#include "types.h"
#include "common_include.h"
#include "config.h"


using namespace std;
using namespace cv;
using namespace MSCKF_MINE;


int main(int argc, char *argv[])
{

    Config::setParameterFile("../config/config.yaml");
    DataReader data;
    vector<CAMERA> vCamera = data.mvCameraData;
    int maxCorners = 300;
    double qualityLevel = 0.01;
    double minDistance = 20;
    int blockSize = 10;
    bool useHarrisDetector = false;
    double k = 0.04;

    Mat feedImage, currentImage;
    bool needFeedImage = true;
    vector<Point2f> corners;
    vector<Point2f> corners_after;

    for(vector<CAMERA>::iterator iter_cam = vCamera.begin(); iter_cam!=vCamera.end(); iter_cam++)
    {
        Mat image = imread(iter_cam->img_name,CV_LOAD_IMAGE_GRAYSCALE);

        /// Parameters for Shi-Tomasi algorithm



        if(needFeedImage)
        {
            needFeedImage = false;
            corners.clear();
            feedImage = image;
            goodFeaturesToTrack( image,
                         corners,
                         maxCorners,
                         qualityLevel,
                         minDistance,
                         Mat(),
                         blockSize,
                         useHarrisDetector,
                         k );

            /// Draw corners detected
            cout<<"** Number of corners detected: "<<corners.size()<<endl;
            cvtColor(image,image,CV_GRAY2BGR);
            int r = 4;
            for( int i = 0; i < corners.size(); i++ )
               { circle( image, corners[i], r, Scalar(0,255,0), -1, 8, 0 ); }



            imshow("Feed Frame", image);
            waitKey(27);
        }
        else
        {
            vector<uchar> status;
            vector<float> err;
            currentImage = image;
            corners_after.clear();
            calcOpticalFlowPyrLK(feedImage,currentImage,corners,corners_after, status, err);
            int j = 0;
            cvtColor(currentImage,currentImage,CV_GRAY2BGR);


            for(int i=0;i<corners_after.size();i++)
            {
                if(status[i]
                        &&((abs(corners[i].x-corners_after[i].x)+abs(corners[i].y-corners_after[i].y))>2)
                        &&((abs(corners[i].x-corners_after[i].x)+abs(corners[i].y-corners_after[i].y))<50))
                {
//                    corners_after[j++] = corners_after[i];
                    j++;
                    cv::circle( currentImage, corners[i], 2, Scalar(0,255,0), -1, 8, 0 );
                    cv::circle( currentImage, corners_after[i], 2, Scalar(0,0,255), -1, 8, 0 );
                    cv::line(currentImage,corners[i],corners_after[i],Scalar(0,255,255),1,8,0);
                }
            }
            cv::imshow("Tracking", currentImage);
            cv::waitKey(27);

            if(j < 150)
                needFeedImage = true;
            cout << "** Run Here --> corners_after.size = " << j << endl;

        }

    }




    return 0;
}

