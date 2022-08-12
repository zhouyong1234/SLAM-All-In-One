/*
 * @Author: your name
 * @Date: 2021-03-26 14:55:20
 * @LastEditTime: 2021-10-10 10:56:06
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/src/DataPretreat/DepthMap.cc
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "DataPretreat/DepthMap.h"
#include "DataPretreat/Config.h"

cv::Mat Depth_tmp;

DepthMap::DepthMap()
{
}

void DepthMap::RGBDDepthBuilder(cv::Mat &imgDepth_, Config &config)
{

    imgDepth_.convertTo(imgDepth, CV_32FC1);
    // std::cout << imgDepth_.type() << std::endl;
    // cv::imwrite("../result/depth.png", imgDepth_);
}

void DepthMap::StereoDepthBuilder(cv::Mat &imgRGBL_, cv::Mat &imgRGBR_, Config &config)
{
    imgRGBL = imgRGBL_.clone();
    imgRGBR = imgRGBR_.clone();
    if(config.ss.MatchingMethod==0){
        imgDepth = SGBM(config, imgRGBL, imgRGBR);
    }

}


cv::Mat DepthMap::SGBM(Config &config, cv::Mat &img_left, cv::Mat &img_right)
{
    int mindisparity = 0;
    int ndisparities = 128;
    int SADWindowSize = 11;
    //SGBM
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    int P1 = 4 * img_left.channels() * SADWindowSize* SADWindowSize;
    int P2 = 32 * img_right.channels() * SADWindowSize* SADWindowSize;

    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(63);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(32);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);
    //sgbm->setMode(cv::StereoSGBM::MODE_HH);

    cv::Mat disp32F;
    sgbm->compute(img_left, img_right, disp32F);
    disp32F.convertTo(disp32F, CV_32F, 1.0/16);

    insertDepth32f(disp32F);

    cv::Mat Depth_Show = cv::Mat(disp32F.rows, disp32F.cols, CV_8UC1);
    cv::normalize(disp32F, Depth_Show, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Mat depthMap = cv::Mat::zeros(disp32F.size(), CV_32FC1);
    int height = disp32F.rows;
    int width = disp32F.cols;
    for(int k = 0;k < height; k++)
    {
        const float* inData = disp32F.ptr<float>(k);
        float* outData = depthMap.ptr<float>(k);
        for(int i = 0; i < width; i++)
        {
            if(!inData[i]) continue;
            outData[i] = float(config.ip.fx *config.ss.baseline / inData[i]);
            // 使每个元素保留3位小数，方便查看
            outData[i] = float(int(outData[i]*1000)/1000);
            if (outData[i]<0 || outData[i]>config.ss.DepthTH)
                outData[i] = -1;
        }
    }
    // 检查深度图
    // CheckDepthMap(depthMap, Depth_Show);

    return depthMap;
}

void DepthMap::insertDepth32f(cv::Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = std::max(0, left);
                right = std::min(right, width - 1);
                top = std::max(0, top);
                bot = std::min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        //cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}

void Mouse_Callback(int event,int x,int y,int flags,void *param){
    if (event == CV_EVENT_LBUTTONDOWN){
        std::cout << "坐标： "<< "x=" << x << "; y=" << y << std::endl;
        cv::Point2f Points_Show;
        Points_Show.x = x;
        Points_Show.y = y;
        float inData = Depth_tmp.ptr<float>((int)Points_Show.y)[(int)Points_Show.x];
        std::cout << "深度（mm）: " << inData << std::endl;
    }

}

double DepthMap::GetDepth(cv::Point2f &coordinate)
{
    double depth;
    depth = imgDepth.ptr<float>((int)coordinate.y)[(int)coordinate.x];
    return depth;
}

cv::Mat DepthMap::GetDepthMap()
{
    return imgDepth;
}

void DepthMap::CheckDepthMap(cv::Mat& depth, cv::Mat &depth_show){
    // CHECK POINT 2
    cv::FileStorage fswrite("../result/DepthMap.xml", cv::FileStorage::WRITE);// 新建文件，覆盖掉已有文件
    fswrite << "src1" << depth;
    fswrite.release();

    // Depth_tmp = depth8U.clone();
    // 注意，这里展示的并不是深度图，因为深度图可视化能力太差，而是用差分图代替
    // 但是鼠标点击显示的数值是深度图的数值
    Depth_tmp = depth.clone();
    cvNamedWindow("depth8U");
    cvSetMouseCallback("depth8U",  Mouse_Callback, 0);
    cv::imshow("depth8U",depth_show);
    cv::waitKey(0);

    cv::imwrite("../result/depth.png",depth);
}

