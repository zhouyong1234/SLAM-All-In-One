/***************************************************************************************
 * Copyright  (c)  2007-2016, ZeroTech Co., Ltd.
 * All rights reserved.
 ***************************************************************************************
 * File name		:  orb_feature.cpp
 * Description		:  
 * Version_info		:  1.0
 * Create Time		:  2019年07月28日 12时01分06秒
 * Author&email		:  zhangxinlu (win), xinlu.zhang@zerotech.com
 * Modify history	:
 ***************************************************************************************
 * Modify Time    Modify person    Modification
 ***************************************************************************************
 *
 ***************************************************************************************/
#include <iostream>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv) 
{ 
    //iMat img_1 = imread("box.png"); 
    //Mat img_2 = imread("box_in_scene.png");
   
	if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );



    // -- Step 1: Detect the keypoints using STAR Detector 
    std::vector<KeyPoint> keypoints_1,keypoints_2;
	int minHessian = 100;
    Ptr<ORB>detector =ORB::create(minHessian); 
    detector->detect(img_1, keypoints_1); 
    detector->detect(img_2, keypoints_2);

    // -- Stpe 2: Calculate descriptors (feature vectors) 
    Mat descriptors_1, descriptors_2; 
    detector->compute(img_1, keypoints_1, descriptors_1); 
    detector->compute(img_2, keypoints_2, descriptors_2);

    //-- Step 3: Matching descriptor vectors with a brute force matcher 
    BFMatcher matcher(NORM_HAMMING); 
    std::vector<DMatch> matches; 
    matcher.match(descriptors_1, descriptors_2, matches); 
    
	// -- dwaw matches 
    Mat img_mathes; 
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_mathes); 
    
	//-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "all points", img_match );
    imshow ( "good_matches", img_goodmatch ); // 中文不能很好识别了

	
	
	// -- show 
    imshow("Mathces", img_mathes);

    waitKey(0); 
    return 0; 
}
