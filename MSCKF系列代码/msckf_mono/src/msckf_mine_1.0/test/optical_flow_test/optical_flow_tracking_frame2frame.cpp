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
    double minDistance = 10;
    int blockSize = 10;
    bool useHarrisDetector = false;
    double k = 0.04;
    bool flag = true;

    Mat feedImage, currentImage, lastImage;
    bool needFeedImage = true;
    vector<Point2f> corners;
    vector<Point2f> corners_after;
    vector<Point2f> corners_before;

    for(vector<CAMERA>::iterator iter_cam = vCamera.begin(); iter_cam!=vCamera.end(); iter_cam++)
    {
        Mat image = imread(iter_cam->img_name, CV_LOAD_IMAGE_GRAYSCALE);
        image.convertTo(image, CV_8UC1);
        std::vector<cv::Mat> image_pyramid;
        buildOpticalFlowPyramid(image, image_pyramid, Size(15,15), 3, true,
                                BORDER_REFLECT_101, BORDER_CONSTANT, false);
        cv::imshow("image_origin", image);
        cv::waitKey(0);
        cout << image_pyramid.size() << endl;
        for(int i = 0; i < image_pyramid.size(); i++)
        {
            if (i % 2 == 0)
            {
                cout << image_pyramid[i].type() << endl;
                cv::imshow("pyrmid", Mat(image_pyramid[i]));
                cv::waitKey(0);
            }




        }


        /*Extract the first frame!*/

        if(needFeedImage)
        {
            needFeedImage = false;
            corners.clear();
            feedImage = image;
            lastImage = image;
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
            if(flag)
            {
                flag = false;
                corners_before = corners;
            }
            cout << "--Tracking information:" << endl;
            cout << "corners_before: " << corners_before.size() << endl;
            calcOpticalFlowPyrLK(lastImage,currentImage,corners_before,corners_after, status, err);
            lastImage = currentImage.clone();

            int j = 0;
            cvtColor(currentImage,currentImage,CV_GRAY2BGR);

            vector<Point2f> corners_before_copy = corners_before;
            corners_before.clear();
            for(int i=0;i<corners_after.size();i++)
            {
                if(status[i]
                        &&((abs(corners_before_copy[i].x-corners_after[i].x)+abs(corners_before_copy[i].y-corners_after[i].y))>=0)
                        &&((abs(corners_before_copy[i].x-corners_after[i].x)+abs(corners_before_copy[i].y-corners_after[i].y))<50))
                {
                    corners_before.push_back(corners_after[i]);
                    j++;

                    //                    cv::circle( currentImage, corners[i], 2, Scalar(0,255,0), -1, 8, 0 );
                    cv::circle( currentImage, corners_after[i], 3, Scalar(0,0,255), -1, 8, 0 );
                    //                    cv::line(currentImage,corners[i],corners_after[i],Scalar(0,255,255),1,8,0);
                }
            }
            cv::imshow("Tracking", currentImage);
            cv::waitKey(27);

            if(j < 100)
            {
                needFeedImage = true;
                flag = true;
            }
            cout << "** Run Here --> corners_after.size = " << j << endl;



        }

    }




    return 0;
}

