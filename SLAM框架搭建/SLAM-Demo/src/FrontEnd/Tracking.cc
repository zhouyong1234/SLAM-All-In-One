/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 14:09:37
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-10-10 10:51:35
 * @Description: file content
 * @FilePath: /SLAM-Demo/src/FrontEnd/Tracking.cc
 */

#include "FrontEnd/Tracking.h"

Tracking::Tracking(Eigen::Matrix4f &initPose_, Config &conf) : initPose(initPose_)
{
    currPose = initPose;
    if(conf.app.img_dataset == "TUM"){
        saveMode = "TUM";
    }else if (conf.app.img_dataset == "KITTI"){
        saveMode = "KITTI";
    }else{
        saveMode = "None";
        std::cerr << "[WARNNING] NO MODE SAVE! " << std::endl;
    }
}

void Tracking::RunTracking(Frame &preFrame, Frame &curFrame, Config &conf)
{
    // Step 1 特征获取及匹配
    std::vector<cv::KeyPoint> KPs1, KPs2;
    cv::Mat Des1, Des2;
    preFrame.GetKPDes(KPs1, Des1);
    curFrame.GetKPDes(KPs2, Des2);

    FeatureManager fm(conf.app);
    std::vector<cv::DMatch> matches;
    fm.FeatureMatch(Des1, Des2, matches);

    // Step 2 位置和深度融合
    cv::Mat preDepth = preFrame.GetDepthMap();
    cv::Mat curDepth = curFrame.GetDepthMap();
    std::vector<cv::Point3f> prePixelPoints, curPixelPoints;
    FusionPosAndDepth(preDepth, curDepth, KPs1, KPs2, Des1, Des2, matches, prePixelPoints, curPixelPoints);
    if (prePixelPoints.size() <= 4)
        return;

    // Step 3 P3P位姿求解及优化
    PoseSolver ps(prePixelPoints, curPixelPoints, conf, 1);
    ps.ComputePnP();


    Eigen::Matrix4f Tcp = ps.GetPose();
    currPose = Tcp * currPose;
    RecordPose(currPose, curFrame.timeStamps);

    // _____CHECK POINT 2_____
    {
        cv::Mat KeyPointShow;
        cv::Mat curimgRGB = curFrame.GetIMGLeft();
        cv::drawKeypoints(curimgRGB, KPs2, KeyPointShow, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
        imshow("KeyPointShow", KeyPointShow);
        cv::waitKey(1);
        if (prePixelPoints.size() < 5 || curPixelPoints.size() < 5)
        {
            std::cout << "[WARRING] Mathes less than 5 pairs." << std::endl;
            cv::waitKey(1);
        }
    }

}

void Tracking::FusionPosAndDepth(
    const cv::Mat &preDepth, const cv::Mat &curDepth,
    const std::vector<cv::KeyPoint> &KPs1, const std::vector<cv::KeyPoint> &KPs2,
    const cv::Mat &Des1, const cv::Mat &Des2,
    const std::vector<cv::DMatch> matches,
    std::vector<cv::Point3f> &prePixelPoints, std::vector<cv::Point3f> &curPixelPoints)
{
    std::vector<cv::KeyPoint> KPs1_Opt, KPs2_Opt;
    for (size_t i(0); i < matches.size(); ++i)
    {
        cv::Point2f prePoint = KPs1[matches[i].queryIdx].pt;
        cv::Point2f curPoint = KPs2[matches[i].trainIdx].pt;
        KPs1_Opt.emplace_back(KPs1[matches[i].queryIdx]);
        KPs2_Opt.emplace_back(KPs2[matches[i].queryIdx]);
        double preDepthPoint = preDepth.ptr<float>((int)prePoint.y)[(int)prePoint.x];
        double curDepthPoint = curDepth.ptr<float>((int)curPoint.y)[(int)curPoint.x];
        if (preDepthPoint <= 0 || curDepthPoint <= 0)
            continue;
        prePixelPoints.emplace_back(prePoint.x, prePoint.y, preDepthPoint);
        curPixelPoints.emplace_back(curPoint.x, curPoint.y, curDepthPoint);
    }

}


void Tracking::RecordPose(Eigen::Matrix4f &T, double &timeStapms){
    ++trajectory.N;
    trajectory.poseDeque.push_back(T);
    trajectory.timeDeque.push_back(timeStapms);
}

void Tracking::SavePose(std::string &filePath)
{
    std::ofstream OutFile(filePath.c_str());
    if (OutFile.is_open())
        OutFile.close();
    for (size_t i=0; i< trajectory.N; ++i){
        const double time = trajectory.timeDeque.at(i);
        const Eigen::Matrix4d Tcw = (trajectory.poseDeque.at(i)).cast<double>();
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block<3, 3>(0, 0) = Tcw.block<3, 3>(0, 0).transpose();
        Twc.block<3, 1>(0, 3) = -Twc.block<3, 3>(0, 0) * Tcw.block<3, 1>(0, 3);

        if (saveMode == "TUM"){
            Eigen::Quaterniond q(Twc.block<3, 3>(0, 0));
            Eigen::Vector3d t(Twc.block<3, 1>(0, 3));
            q.normalized();

            std::ofstream OutFile(filePath.c_str(), std::ios::app);

            if (OutFile.is_open())
            {
                OutFile << std::fixed << std::setprecision(6)
                << std::to_string(time) << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
                OutFile.close();
            }
        }else if (saveMode == "KITTI")
        {
            std::ofstream OutFile(filePath.c_str(), std::ios::app);

            if (OutFile.is_open())
            {
                OutFile << std::fixed << std::setprecision(9)
                << Twc(0, 0) << " " << Twc(0, 1) << " "
                << Twc(0, 2) << " " << Twc(0, 3) << " "
                << Twc(1, 0) << " " << Twc(1, 1) << " "
                << Twc(1, 2) << " " << Twc(1, 3) << " "
                << Twc(2, 0) << " " << Twc(2, 1) << " "
                << Twc(2, 2) << " " << Twc(2, 3)
                // << Tcw(3, 0) << " " << Tcw(3, 1) << " "
                // << Tcw(3, 2) << " " << Tcw(3, 3)
                << std::endl;
                OutFile.close();
            }
        }
    }
}

Tracking::~Tracking()
{
}
