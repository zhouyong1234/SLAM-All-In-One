#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cassert>
#include <chrono>
#include "DataPretreat/Config.h"
#include "DataPretreat/DepthMap.h"
#include "DataPretreat/Frame.h"
#include "FrontEnd/FeatureManager.h"
#include "FrontEnd/Tracking.h"
#include "BackEnd/Optimization.h"

using namespace std;
using namespace cv;

void LoadImagesStereo(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<string> &vTimestamps);

void LoadImagesRGBD(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<string> &vTimestamps);

int main(int argc, char const *argv[])
{

    // Part 1：读取参数配置文件，读取图片及预处理
    if ( argc != 2){
        cerr << "[ERROR] Please check argv! " << endl;
        return -1;
    }


    // Step 1_1 读取参数文件
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "[ERROR] Failed to open settings file at: " << argv[1] << endl;
        exit(-1);
    }
    Config conf(fsSettings);
    cout << "[INFO] Loading the parameter files...." << endl;


    // Step 1_2 读取图像序列地址
    vector<string> vstrImageLeft, vstrImageRight, vTimestamps;
    if (conf.app.img_dataset == "KITTI")
        LoadImagesStereo(conf.app.img_path, vstrImageLeft, vstrImageRight, vTimestamps);
    else if (conf.app.img_dataset == "TUM")
        LoadImagesRGBD(conf.app.img_path, vstrImageLeft, vstrImageRight, vTimestamps);
    else
        cout << "[ERROR] Only Support KITTI and TUM! " << endl;
    const int nImages = vstrImageLeft.size();

    cout << "Total have " << nImages << " images in the sequence" << endl;

    cv::Mat imgRGBL_0, imgRGBR_0, imgDepth_0;
    cv::Mat imgRGBL_1, imgRGBR_1, imgDepth_1;
    imgRGBL_0 = cv::imread(vstrImageLeft[0], CV_LOAD_IMAGE_UNCHANGED);
    imgRGBR_0 = cv::imread(vstrImageRight[0], CV_LOAD_IMAGE_UNCHANGED);
    Frame preFrame(imgRGBL_0, imgRGBR_0, conf, vTimestamps[0]);

    Eigen::Matrix4f Tcw = Eigen::Matrix4f::Identity();
    Tracking tracker(Tcw, conf);

    // Part 2：对每相邻两帧图像进行匹配和位姿计算
    for(size_t i=1; i<nImages; i++)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cout << "===>> Current Frame: " << vTimestamps[i] << endl;
        imgRGBL_1 = cv::imread(vstrImageLeft[i], CV_LOAD_IMAGE_UNCHANGED);
        imgRGBR_1 = cv::imread(vstrImageRight[i], CV_LOAD_IMAGE_UNCHANGED);
        if (imgRGBL_1.empty() || imgRGBR_1.empty())
        {
            cerr << "[ERROR] Please check the path of images!" << endl;
            continue;
        }
        // 生成当前帧
        Frame curFrame(imgRGBL_1, imgRGBR_1, conf, vTimestamps[i]);


        // 运行追踪线程
        tracker.RunTracking(preFrame, curFrame,conf);

        // 将当前帧置为上一帧
        preFrame = curFrame;

        // waitKey(1);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "[INFO] costs time: " << time_used.count() << " seconds." << std::endl;
    }

    tracker.SavePose(conf.app.pose_path);

    return 0;
}

void LoadImagesStereo(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<string> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    string s;
    while(!fTimes.eof())
    {
        getline(fTimes,s);
        stringstream ss;
        ss << s;
        double nt;
        ss >> nt;
        std::string st = to_string(nt);
        vTimestamps.push_back(st);
    }

    const int nTimes = 4541;
    // const int nTimes = vTimestamps.size();
    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft.push_back(strPrefixLeft + ss.str() + ".png");
        vstrImageRight.push_back(strPrefixRight + ss.str() + ".png");
    }
    cout << vstrImageLeft.size() << endl;
}

void LoadImagesRGBD(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<string> &vTimestamps)
{
    ifstream fAssociation;
    const string associationFile = strPathToSequence + "associate.txt";
    fAssociation.open(associationFile.c_str());
    cout << "[LOAD] Open Association File ..." << endl;
    while (!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            string st, sRGB, sD;
            ss >> st;
            vTimestamps.emplace_back(st);
            ss >> sRGB;
            vstrImageFilenamesRGB.emplace_back(strPathToSequence + sRGB);
            ss >> st;
            ss >> sD;
            vstrImageFilenamesD.emplace_back(strPathToSequence + sD);
        }
    }
}
