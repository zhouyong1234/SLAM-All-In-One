#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <thread>
#include "OpticalSLAM.h"
#include "params.h"

using namespace cv;
// using namespace cv::xfeatures2d;


int main()
{
    cv::Mat image_color = cv::imread("/home/touchair/图片/2021/10/28/IMG_20211028_161619.jpg", cv::IMREAD_COLOR);

    cv::resize(image_color, image_color, cv::Size(image_color.cols/4, image_color.rows/4));

    cv::Mat image_copy = image_color.clone();
 
	//使用灰度图像进行角点检测
	cv::Mat image_gray, image_down;
	cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
	cv::pyrDown(image_gray, image_down, cv::Size(image_gray.cols/2, image_gray.rows/2));
 
	//设置角点检测参数
	std::vector<cv::Point2f> corners;
	int max_corners = 100;
	double quality_level = 0.01;
	double min_distance = 5.0;
	int block_size = 3;
	bool use_harris = false;
	double k = 0.04;
 
	//角点检测
	cv::goodFeaturesToTrack(image_gray, 
							corners, 
							max_corners, 
							quality_level, 
							min_distance, 
							cv::Mat(), 
							block_size, 
							use_harris, 
							k);
 
	//将检测到的角点绘制到原图上
	for (int i = 0; i < corners.size(); i++)
	{
        // std::cout << corners[i] << std::endl;
		cv::circle(image_color, corners[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
	}

    //亚像素检测
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 40, 0.01);
    cv::cornerSubPix(image_gray, corners, cv::Size(5,5), cv::Size(-1,-1), criteria);

    for(int i = 0; i < corners.size(); i++)
    {
        cv::circle(image_copy, corners[i], 5, cv::Scalar(0,255,0), 2, 8, 0);
    }


	
	vector<KeyPoint>detectKeyPoint, detectKeyPoint1, detectKeyPoint2;
    Mat keyPointImage1, keyPointImage2, keyPointImage3;
	Ptr<GFTTDetector> gftt = cv::GFTTDetector::create(300, 0.1, 20);
	gftt->detect(image_gray,detectKeyPoint);
	drawKeypoints(image_gray,detectKeyPoint,keyPointImage1,Scalar(0,0,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	drawKeypoints(image_gray,detectKeyPoint,keyPointImage2,Scalar(0,0,255),DrawMatchesFlags::DEFAULT);
	imshow("keyPoint image1",keyPointImage1);
	imshow("keyPoint image2",keyPointImage2);


	Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
	fast->detect(image_gray, detectKeyPoint1);
	drawKeypoints(image_color,detectKeyPoint1,keyPointImage3,Scalar(0,0,255),DrawMatchesFlags::DEFAULT);
	imshow("keyPoint image3",keyPointImage3);


	// cv::Ptr<SiftFeatureDetector> dog = SiftFeatureDetector::create(1000);


 
	cv::imshow("corner", image_color);
    cv::imshow("sub pixel corner", image_copy);

	cv::imshow("pyrDown", image_down);

	cv::waitKey(0);
	return 0;
}