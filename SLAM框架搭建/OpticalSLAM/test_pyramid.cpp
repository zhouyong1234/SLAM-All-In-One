#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <thread>
#include "OpticalSLAM.h"
#include "params.h"


std::vector<cv::Mat> curr_pyramid;


int main() {
    cv::Mat src = cv::imread("/home/touchair/下载/dataset/data_odometry_color/00/image_2/004540.png", cv::IMREAD_COLOR);

    cv::Mat image_gray;
    cv::cvtColor(src, image_gray, cv::COLOR_BGR2GRAY);

    // cv::resize(src, src, cv::Size(src.cols/3, src.rows/3));
    cv::buildOpticalFlowPyramid(src, curr_pyramid, cv::Size(21, 21), 6, true, BORDER_REFLECT101, BORDER_CONSTANT, false);


    std::cout << curr_pyramid.size() << std::endl;
    cv::Mat curr_pyramid_gray;
    cv::cvtColor(curr_pyramid[0], curr_pyramid_gray, cv::COLOR_BGR2GRAY);
    

    
    std::vector<cv::Point2f> corners;
	int max_corners = 200;
	double quality_level = 0.01;
	double min_distance = 30.0;
 
	//角点检测
	cv::goodFeaturesToTrack(image_gray, 
							corners, 
							max_corners, 
							quality_level, 
							min_distance);
 
	//将检测到的角点绘制到原图上
	for (int i = 0; i < corners.size(); i++)
	{
        // std::cout << corners[i] << std::endl;
		cv::circle(src, corners[i], 2, cv::Scalar(0, 0, 255), 2, 8, 0);
	}

    //角点检测
	cv::goodFeaturesToTrack(curr_pyramid_gray, 
							corners, 
							max_corners, 
							quality_level, 
							min_distance);
 
	//将检测到的角点绘制到原图上
	for (int i = 0; i < corners.size(); i++)
	{
        // std::cout << corners[i] << std::endl;
		cv::circle(curr_pyramid[0], corners[i], 2, cv::Scalar(255, 255, 0), 2, 8, 0);
	}
    // cv::imshow("curr_pyramid[1]", curr_pyramid[1]);
    // cv::imshow("curr_pyramid[2]", curr_pyramid[2]);
    // cv::imshow("curr_pyramid[3]", curr_pyramid[3]);
    // cv::imshow("curr_pyramid[4]", curr_pyramid[4]);
    // cv::imshow("curr_pyramid[5]", curr_pyramid[5]);

    // cv::Mat gray, dst;
    // cv::cvtColor(src, gray, COLOR_BGR2GRAY);
    // cv::imshow("gray", gray);
    // cv::equalizeHist(gray, dst);
    // cv::imshow("dst", dst);

    cv::imshow("src", src);
    cv::imshow("curr_pyramid[0]", curr_pyramid[0]);


    cv::waitKey(0);
    
}