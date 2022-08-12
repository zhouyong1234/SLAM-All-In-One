#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
int main() {
  cv::Mat test_rgb = cv::Mat(20,20,CV_8UC3);
  for (int i = 0; i < test_rgb.rows; ++i) {
    for (int j = 0; j < test_rgb.cols; ++j) {
      test_rgb.at<cv::Vec3b>(i,j)[0] = 255;
      test_rgb.at<cv::Vec3b>(i,j)[1] = 100;
      test_rgb.at<cv::Vec3b>(i,j)[2] = 0;
    }
  }
  std::cout << "Data: \n" << test_rgb  << std::endl;
  cv::imshow("test_3channel",test_rgb);
  cv::waitKey(0);
  std::cout << "KongDqing Hello!" << std::endl;
  return 0;
}