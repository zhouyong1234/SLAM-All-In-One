/*
 * @Author: your name
 * @Date: 2021-03-26 11:25:51
 * @LastEditTime: 2021-10-09 16:06:35
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/include/DataPretreat/Config.h
 */
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <iostream>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>

class Config
{
public:
    struct AppSettings
    {
        std::string img_dataset;
        std::string img_path;
        std::string Mode;
        std::string pose_path;
        int ORB_Features;
        int SIFT_Features;
        int SURF_Features;
        int Resize;
    };

    struct StereoSetting
    {
        int MatchingMethod;
        float baseline;
        float DepthTH;
    };

    struct InternalParameters
    {
        float fx, fy, cx, cy;
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        cv::Mat DistCoef = cv::Mat::zeros(4,1,CV_32F);
    };

    struct OptimizationConfig
    {
        int maxRansacIter;
        double ErrorTH;
    };



public:
    cv::FileStorage SettingsFile;

    AppSettings app;
    StereoSetting ss;
    InternalParameters ip;
    OptimizationConfig oc;

public:
    Config(cv::FileStorage &fsSettings):SettingsFile(fsSettings)
    {
        AppSettingsInit();

        InternalParametersInit();
        OptimizationConfigInit();

        if (app.Mode == "Stereo")
            StereoSettingInit();

    };

    void AppSettingsInit(){
        app.img_dataset = static_cast<std::string>(SettingsFile["Image.Dataset"]);
        app.img_path = static_cast<std::string>(SettingsFile["Image.Path"]);
        app.Mode = static_cast<std::string>(SettingsFile["Image.Mode"]);
        app.ORB_Features = SettingsFile["Image.ORB_Features"];
        app.SIFT_Features = SettingsFile["Image.SIFT_Features"];
        app.SURF_Features = SettingsFile["Image.SURF_Features"];
        app.Resize = SettingsFile["Image.Resize"];

        app.pose_path = static_cast<std::string>( SettingsFile["Pose.Path"]);
    };

    void StereoSettingInit(){
        ss.baseline = SettingsFile["Stereo.bf"];
        ss.DepthTH = SettingsFile["Stereo.th"];
        ss.MatchingMethod = SettingsFile["Stereo.MatchingMethod"];
    };

    void InternalParametersInit(){
        ip.fx = SettingsFile["Camera.fx"];
        ip.fy = SettingsFile["Camera.fy"];
        ip.cx = SettingsFile["Camera.cx"];
        ip.cy = SettingsFile["Camera.cy"];
        ip.K.at<float>(0,0) = ip.fx;
        ip.K.at<float>(1,1) = ip.fy;
        ip.K.at<float>(0,2) = ip.cx;
        ip.K.at<float>(1,2) = ip.cy;
        ip.DistCoef.at<float>(0) = SettingsFile["Camera.k1"];
        ip.DistCoef.at<float>(1) = SettingsFile["Camera.k2"];
        ip.DistCoef.at<float>(2) = SettingsFile["Camera.p1"];
        ip.DistCoef.at<float>(3) = SettingsFile["Camera.p2"];
    };

    void OptimizationConfigInit(){
        oc.maxRansacIter = SettingsFile["Opts.maxRansacIter"];
        oc.ErrorTH = SettingsFile["Opts.ErrorTH"];
    }

};

#endif //CONFIG_H

