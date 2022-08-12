//
// Created by kdq on 2021/10/8.
//
#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class ImageGrayValueHistogram {
 public:
  ImageGrayValueHistogram() = delete;
  ImageGrayValueHistogram(int fastThreshold,int gridCols = 2,int gridRows = 2,int downSample = 1):
    fastThreshold_(fastThreshold),
    gridCols_(gridCols),
    gridRows_(gridRows),
    downSample_(downSample) {
  }
  float CalculateGridsGrayValue(cv::Mat& image,std::vector<std::vector<float>>& gridValues,std::vector<std::vector<int>>& cornerSizes,
                                int minCornerSize = 10,int minGrayValue = 50) {
    int goodPatchForDetect = 0;
    if (image.empty()) {
      return goodPatchForDetect;
    }
    cv::Mat img;
    cv::resize(image,img,cv::Size(image.cols/downSample_,image.rows/downSample_),0,0,CV_INTER_AREA);
    int cellHeight = floor(img.rows / gridRows_);
    int cellWidth = floor(img.cols / gridCols_);
    gridValues.resize(gridRows_,std::vector<float>(gridCols_));
    cornerSizes.resize(gridRows_,std::vector<int>(gridCols_));
    for (int i = 0; i < gridRows_; ++i) {
      for (int j = 0; j < gridCols_; ++j) {
        int x = j * cellWidth;
        int y = i * cellHeight;
        cv::Mat tmp(img,cv::Rect(x,y,cellWidth,cellHeight));
        std::vector<cv::KeyPoint> pts;
        cv::FAST(tmp,pts,fastThreshold_,true);
        gridValues[i][j] = cv::mean(tmp)[0];
        cornerSizes[i][j] = pts.size();
        if (cornerSizes[i][j] > minCornerSize && gridValues[i][j] > minGrayValue) {
          goodPatchForDetect++;
        }
      }
    }
    return (float)(goodPatchForDetect) / (gridRows_ * gridCols_);
  }
 private:
  const int fastThreshold_;
  const int gridCols_;
  const int gridRows_;
  const int downSample_;
};