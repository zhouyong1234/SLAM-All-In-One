#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <thread>
#include "OpticalSLAM.h"
#include "params.h"



int main() {
    cv::Mat src = cv::imread("/home/touchair/图片/2021/10/28/IMG_20211028_161619.jpg", cv::IMREAD_COLOR);

    cv::resize(src, src, cv::Size(src.cols/3, src.rows/3));

    cv::imshow("src", src);

    cv::Mat gray, dst;
    cv::cvtColor(src, gray, COLOR_BGR2GRAY);
    cv::imshow("gray", gray);
    cv::equalizeHist(gray, dst);
    cv::imshow("dst", dst);

    cv::waitKey(0);
    
}