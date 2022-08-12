#include "ImageAlignment.hpp"


int main(int argc,char** argv) {
  if (argc != 3) {
    std::cout << "please input image 1 and image 2 file!" << std::endl;
    return -1;
  }
  std::string image1 = argv[1];
  std::string image2 = argv[2];
  // Read the images to be aligned 读取仿射图像
  //im1参考图像，im2要处理的图像
  cv::Mat im1 = imread(image1,cv::IMREAD_GRAYSCALE);
  cv::Mat im2 = imread(image2,cv::IMREAD_GRAYSCALE);
  cv::Mat halfIm1,halfIm2;
  cv::pyrDown(im1,halfIm1,im1.size() / 2);
  cv::pyrDown(im2,halfIm2,im1.size() / 2);

  int alignModel = cv::MOTION_EUCLIDEAN;
  ImageAlignment imgAlign(alignModel,40,1e-6,false);
  const double toc_start  = (double) cv::getTickCount ();
  cv::Mat warp12 = imgAlign.Align(0,halfIm1,halfIm2);
  const double toc_final  = ((double) cv::getTickCount () - toc_start) / cv::getTickFrequency();
  std::cout << "Cost time = " << toc_final << std::endl;
  cv::Mat warp21 = imgAlign.Align(0,halfIm2,halfIm1);
  cv::Mat warpCheck;
  if (alignModel == cv::MOTION_HOMOGRAPHY) {
    warpCheck = warp12 * warp21;
  } else {
    cv::Mat tm1 = cv::Mat::eye(3,3,CV_32F);
    cv::Mat tm2 = cv::Mat::eye(3,3,CV_32F);
    for (size_t i = 0; i < 2; i++) {
      for (size_t j = 0; j < 3; j++) {
        tm1.at<float>(i,j) = warp12.at<float>(i,j);
        tm2.at<float>(i,j) = warp21.at<float>(i,j);
      }
    }
    std::cout << "tm1 = \n" << tm1 << std::endl;
    warpCheck = tm1 * tm2;
  }
  std::cout << "Warp12 = \n" << warp12 << std::endl;
  std::cout << "Warp21 = \n" << warp21 << std::endl;
  std::cout << "CheckWarp:\n" << warpCheck << std::endl;
  return 0;
}
