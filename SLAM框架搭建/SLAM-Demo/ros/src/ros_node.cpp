/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-07 21:22:34
 * @LastEditTime: 2021-10-07 22:52:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/ros/ros_node.cpp
 */
#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cassert>
#include <chrono>
#include "../DataPretreat/Config.h"
#include "../DataPretreat/DepthMap.h"
#include "../FrontEnd/FeatureMather.h"
#include "../FrontEnd/PoseSolver.h"
#include "../BackEnd/Optimization.h"

using namespace std;
using namespace cv;

void LoadImagesStereo(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<string> &vTimestamps);

void LoadImagesRGBD(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<string> &vTimestamps);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_node");
    ros::NodeHandle nh;

    // Part 1：读取参数配置文件，读取图片及预处理
    if (argc != 2)
    {
        cerr << "[ERROR] Please check argv! " << endl;
        return -1;
    }

    // Step 1_1 读取参数文件
    std::cout << argv[1] << std::endl;
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if (!fsSettings.isOpened())
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

    // Part 2：对每相邻两帧图像进行匹配和位姿计算
    for (size_t i = 0; i < nImages - 1; i++)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Step 2_1 读取图像（并裁剪），转化为单通道图像
        cv::Mat imgBGR_0 = cv::imread(vstrImageLeft[i], CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat imgBGR_1 = cv::imread(vstrImageLeft[i + 1], CV_LOAD_IMAGE_UNCHANGED);
        if (imgBGR_0.empty() || imgBGR_1.empty())
        {
            cerr << "[ERROR] Please check the path of images!" << endl;
            return -1;
        }
        cout << "===>> Current Frame: " << vTimestamps[i] << endl;
        cv::Mat imgRize_0, imgRize_1, imgGray_0, imgGray_1;
        if (conf.app.Resize == true)
        {
            resize(imgBGR_0, imgRize_0, cv::Size(640, 480));
            resize(imgBGR_1, imgRize_1, cv::Size(640, 480));
        }
        else
        {
            imgRize_0 = imgBGR_0;
            imgRize_1 = imgBGR_1;
        }
        cvtColor(imgRize_0, imgGray_0, COLOR_BGR2GRAY);
        cvtColor(imgRize_1, imgGray_1, COLOR_BGR2GRAY);
        // // CHECK POINT 1
        // imshow("input_image_1",imgRize_0);
        // waitKey(1);
        // imshow("input_image_2",imgRize_1);
        // waitKey(0);

        // Step 2_2 特征检测与匹配
        std::vector<KeyPoint> Kp1, Kp2;
        std::vector<DMatch> matches;
        FeatureMatcher fm(conf.app, imgGray_0, imgGray_1);
        fm.FeatureExtraction(matches, Kp1, Kp2);

        // Step 2_3 深度图构建
        DepthMap dm(imgGray_0, imgGray_1);

        if (conf.app.Mode == "Stereo")
        {
            // Step 2_3_1 如果是立体相机，则sgm算法生成深度图
            cv::Mat imgBGR_0_r = cv::imread(vstrImageRight[i]);
            cv::Mat imgBGR_1_r = cv::imread(vstrImageRight[i + 1]);
            cv::Mat imgRize_0_r, imgRize_1_r, imgGray_0_r, imgGray_1_r;
            if (conf.app.Resize == true)
            {
                resize(imgBGR_0_r, imgRize_0_r, cv::Size(640, 480));
                resize(imgBGR_1_r, imgRize_1_r, cv::Size(640, 480));
            }
            else
            {
                imgRize_0_r = imgBGR_0_r;
                imgRize_1_r = imgBGR_1_r;
            }
            cvtColor(imgRize_0_r, imgGray_0_r, COLOR_BGR2GRAY);
            cvtColor(imgRize_1_r, imgGray_1_r, COLOR_BGR2GRAY);

            cv::Mat imgGray_0_l = imgGray_0;
            cv::Mat imgGray_1_l = imgGray_1;

            dm.StereoDepthBuilder(imgGray_0_r, imgGray_1_r, conf);
        }
        else if (conf.app.Mode == "RGBD")
        {
            // Step 2_3_2 如果是RGBD相机，则直接读取深度图
            cv::Mat imgDepth_0_l = cv::imread(vstrImageRight[i], CV_LOAD_IMAGE_UNCHANGED);
            cv::Mat imgDepth_1_l = cv::imread(vstrImageRight[i + 1], CV_LOAD_IMAGE_UNCHANGED);
            dm.RGBDDepthBuilder(imgDepth_0_l, imgDepth_1_l, conf);
        }
        else
        {
            cout << "[ERROR] We Only Support Stereo or RGBD." << endl;
        }

        cv::Mat imgDepth_0_left = dm.GetDepthMap_0();
        cv::Mat imgDepth_1_left = dm.GetDepthMap_1();
        // std::cout << "[INFO] DepthMap Build Finished! " << std::endl;

        // Step 2_3_3 计算三维空间点
        std::vector<cv::Point3f> PixelPoint3fVec_0, PixelPoint3fVec_1;
        std::vector<cv::KeyPoint> Kp1_Opt, Kp2_Opt;
        for (size_t i(0); i < matches.size(); ++i)
        {
            cv::Point2f tmpPoint_0 = Kp1[matches[i].queryIdx].pt;
            cv::Point2f tmpPoint_1 = Kp2[matches[i].trainIdx].pt;
            Kp1_Opt.emplace_back(Kp1[matches[i].queryIdx]);
            Kp2_Opt.emplace_back(Kp2[matches[i].queryIdx]);
            double depth_0 = dm.GetDepth(tmpPoint_0, 0);
            double depth_1 = dm.GetDepth(tmpPoint_1, 1);
            if (depth_0 <= 0 || depth_1 <= 0)
                continue;
            PixelPoint3fVec_0.emplace_back(tmpPoint_0.x, tmpPoint_0.y, depth_0);
            PixelPoint3fVec_1.emplace_back(tmpPoint_1.x, tmpPoint_1.y, depth_1);
        }
        // // CHECK POINT 2
        cv::Mat KeyPointShow;
        // drawKeypoints(imgRize_0, Kp1_Opt, KeyPointShow, cv::Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(imgRize_1, Kp2_Opt, KeyPointShow, cv::Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
        imshow("KeyPointShow", KeyPointShow);
        if (PixelPoint3fVec_0.size() < 6 || PixelPoint3fVec_1.size() < 6)
        {
            cout << "[WARRING] Mathes less than 6 pairs." << endl;
            waitKey(0);
            continue;
        }

        // Step 2_4 P3P位姿求解及优化
        PoseSolver ps(PixelPoint3fVec_0, PixelPoint3fVec_1, conf, 1);
        ps.ComputePnP();

        // // EpipolarLine el;
        waitKey(1);

        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "[INFO] costs time: " << time_used.count() << " seconds." << std::endl;
    }

    return EXIT_SUCCESS;
}

void LoadImagesStereo(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<string> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    string s;
    while (!fTimes.eof())
    {
        getline(fTimes, s);
        stringstream ss;
        ss << s;
        string st;
        ss >> st;
        vTimestamps.push_back(st);
    }

    const int nTimes = 20;
    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";
    for (int i = 0; i < nTimes; i++)
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
