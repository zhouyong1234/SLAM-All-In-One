/*
 * @Author: your name
 * @Date: 2021-03-26 09:03:16
 * @LastEditTime: 2021-10-08 17:07:54
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/src/FrontEnd/FeatureMather.cc
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>
#include <mutex>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "FeatureMather.h"
#include "DataPretreat/Config.h"

// #define DEBUG
#ifdef DEBUG
#define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
#define CHECK_INFO_2(x, y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
#define CHECK_INFO(x)      //std::cout << x << std::endl;
#define CHECK_INFO_2(x, y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

FeatureMatcher::FeatureMatcher(Config::AppSettings conf, cv::Mat &Img_0, cv::Mat &Img_1)
    : Frame_0(Img_0),Frame_1(Img_1), app(conf)
{
}

void FeatureMatcher::FeatureExtraction(std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2){
         if (app.ORB_Features == true)
            OrbExtract( matches, Kp1, Kp2);
        else if (app.SIFT_Features == true)
            SiftExtract( matches, Kp1, Kp2);
        else if (app.SURF_Features == true)
            SurfExtract( matches, Kp1, Kp2);
        else
            std::cerr << "[ERRO] NO Extraction Method" << std::endl;
}

void FeatureMatcher::OptimizedDec(std::vector<cv::DMatch> &matches, const int &row, double th, double coefficient)
{
    std::vector< cv::DMatch > Gmatches;
    double min_dist=10000, max_dist=0;
    for ( int i(0); i <row ; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    //cout << "max :" << max_dist << endl << "min :" << min_dist << endl;
    for ( int i = 0; i < row ; i++ )
    {
        if ( matches[i].distance <= std::max( coefficient*min_dist, th ) )
        {
            Gmatches.push_back ( matches[i] );
        }
    }
    matches = Gmatches;
}

void FeatureMatcher::SurfExtract( std::vector<cv::DMatch> &matches,
    std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2 )
{
    cv::Mat Des1, Des2;
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create(500);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::xfeatures2d::SURF::create();
    //sift，surf，为CV32F直接套用汉明匹配则报错
    cv::FlannBasedMatcher matcher;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    detector->detect ( Frame_0, Kp1 );
    detector->detect ( Frame_1, Kp2 );
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double t01= std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    CHECK_INFO_2("SURF 关键点检测耗时： ", t01);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    descriptor->compute ( Frame_0, Kp1, Des1 );
    descriptor->compute ( Frame_1, Kp2, Des2 );
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double t23= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    CHECK_INFO_2("SURF 描述子检测耗时： ", t23);

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    matcher.match( Des1, Des2, matches );
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    double t45= std::chrono::duration_cast<std::chrono::duration<double> >(t5 - t4).count();
    CHECK_INFO_2("SURF 匹配检测耗时： ", t23 );

    CHECK_INFO_2("优化前匹配数量： ", matches.size() );
    OptimizedDec(matches, Des1.rows, 0.01, 3);
    // cv::Mat ShowMatches;
    // cv::drawMatches ( Frame_0, Kp1, Frame_1, Kp2, matches, ShowMatches, cv::Scalar(0,255,0),cv::Scalar(0,255,0) );
    // cv::imwrite( "../result/SURF.png", ShowMatches);
    CHECK_INFO_2("优化后匹配数量： ", matches.size() );
    // cv::imshow ( "SURF: ShowMatches", ShowMatches );
    // cv::waitKey(0);

}

void FeatureMatcher::SiftExtract( std::vector<cv::DMatch> &matches,
    std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2 )
{
    cv::Mat Des1, Des2;
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create(500);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::xfeatures2d::SIFT::create();
    //sift，surf，为CV32F直接套用汉明匹配则报错
    cv::FlannBasedMatcher matcher;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    detector->detect ( Frame_0, Kp1 );
    detector->detect ( Frame_1, Kp2 );
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double t01= std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    CHECK_INFO_2("SIFT 关键点检测耗时： ", t01 );

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    descriptor->compute ( Frame_0, Kp1, Des1 );
    descriptor->compute ( Frame_1, Kp2, Des2 );
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double t23= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    CHECK_INFO_2("SIFT 描述子检测耗时： ", t23 );

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    matcher.match( Des1, Des2, matches );
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    double t45= std::chrono::duration_cast<std::chrono::duration<double> >(t5 - t4).count();
    CHECK_INFO_2("SIFT 匹配检测耗时： ", t23 );

    CHECK_INFO_2("优化前匹配数量： ", matches.size() );
    OptimizedDec(matches, Des1.rows, 100,2);
    CHECK_INFO_2("优化后匹配数量： ", matches.size() );

    // cv::Mat ShowMatches;
    // cv::drawMatches ( Frame_0, Kp1, Frame_1, Kp2, matches, ShowMatches, cv::Scalar(0,255,0),cv::Scalar(0,255,0) );
    // cv::imwrite( "../result/SIFT.png", ShowMatches);
    // cv::imshow ( "SIFT: ShowMatches", ShowMatches );
    // cv::waitKey(0);

}

void FeatureMatcher::OrbExtract( std::vector<cv::DMatch> &matches,
    std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2 )
{
    cv::Mat Des1, Des2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,20);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-HammingLUT" );
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    detector->detect ( Frame_0, Kp1 );
    detector->detect ( Frame_1, Kp2 );

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double t01= std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    CHECK_INFO_2("ORB 关键点检测耗时： ", t01 );

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    descriptor->compute ( Frame_0, Kp1, Des1 );
    descriptor->compute ( Frame_1, Kp2, Des2 );
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double t23= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    CHECK_INFO_2("ORB 描述子检测耗时： ", t23 );

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    matcher->match( Des1, Des2, matches );
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    double t45= std::chrono::duration_cast<std::chrono::duration<double> >(t5 - t4).count();
    CHECK_INFO_2("ORB 匹配检测耗时： ", t23 );

    CHECK_INFO_2("优化前匹配数量： ", matches.size() );
    OptimizedDec(matches, Des1.rows, 30.0, 2);
    CHECK_INFO_2("优化后匹配数量： ", matches.size() );
    // cv::Mat ShowMatches;
    // cv::drawMatches ( Frame_0, Kp1, Frame_1, Kp2, matches, ShowMatches, cv::Scalar(0,255,0),cv::Scalar(0,255,0) );
    // cv::imwrite( "../result/ORB.png", ShowMatches);
    // cv::imshow ( "ORB: ShowMatches", ShowMatches );

    // cv::waitKey(0);
}

