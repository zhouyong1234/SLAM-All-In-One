#include <iostream>
#include <chrono>


using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc.hpp>
#include <vector>
#include <opencv/cv.h>



int main ( int argc, char** argv)
{
    cv::Mat image;
    image = cv::imread( argv[1]);
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,cv::COLOR_RGB2GRAY);

    
    //设置角点检测参数
    vector<cv::Point2f>corners;
    int max_corners = 200;
    double quality_lever = 0.01;
    double min_dis = 3.0;
    int block_size= 3;
    bool use_harris = false;
    double k = 0.04;

    if ( image.data == nullptr )
    {
	cerr<<"文件"<<argv[1]<<"不存在."<<endl;
	return 0;
    }

    cout<<"图像宽为"<<image.cols<<"，高为"<<image.rows<<",通道数为"<<image.channels()<<endl;
    cv::imshow( " image",image );
    cv::waitKey( 0 );

    if ( image.type() != CV_8UC1 && image.type() != CV_8UC3 )
    {
	cerr<<"文件"<<argv[1]<<"不存在"<<endl;
	return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    cv::goodFeaturesToTrack(image_gray,corners,max_corners,quality_lever,min_dis,cv::Mat(),block_size,use_harris,
	    k);

    for( int i=0;i<corners.size();i++)
    {
	cv::circle(image,corners[i],1,cv::Scalar(0,0,255),2,8,0);
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double>time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1);

    cout<<"获取角点用时："<<time_used.count()<<"秒."<<endl;

    
    //cv::Mat image_another = image;
    //image_another (cv::Rect ( 0,0,100,100)).setTo ( 0 ); //
    //cv::imshow( "image",image);
    //cv::waitKey( 0 );


    cv::Mat image_clone = image.clone();
   // image_clone (cv::Rect (500,200,100,100)).setTo(155);
   // cv::imshow("image",image);
    cv::imshow("image_clone",image_clone);
    cv::waitKey( 0 );

    cv::destroyAllWindows();
    return 0;

}
