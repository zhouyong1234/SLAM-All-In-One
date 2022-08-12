#include <iostream>
#include "ImageGrayValueHistogram.hpp"

int main(int argc,char** argv) {
  if (argc != 2) {
    std::cout << "Please input one image" << std::endl;
    return -1;
  }
  std::string imageFile = argv[1];
  cv::Mat image = cv::imread(imageFile,cv::IMREAD_GRAYSCALE);
  ImageGrayValueHistogram gryCalHist(5,4,3,1);
  std::vector<std::vector<float>> histogram;
  std::vector<std::vector<int>> cornerSize;
  float percent = gryCalHist.CalculateGridsGrayValue(image,histogram,cornerSize);
  std::cout << "Good Percent:" << percent << std::endl;
  for (int i = 0; i < histogram.size(); ++i) {
    int j = 0;
    for (auto v: histogram[i]) {
      printf("<%d,%d>:%f,%d  ", i,j,v,cornerSize[i][j]);
      j++;
    }
    std::cout << std::endl;
  }
  cv::imshow("image",image);
  while(cv::waitKey(0) != 'q') {

  }
}