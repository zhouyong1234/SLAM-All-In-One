#define BACKWARD_HAS_DW 1
#include"code_utils/backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include <code_utils/sys_utils/tic_toc.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;
using namespace cv;

double t;

void
showImg( std::string name, cv::Mat img )
{
    cv::namedWindow( name, WINDOW_NORMAL );
    cv::imshow( name, img );
}

void
sumPixelRow( const cv::Mat& img, cv::Mat& integral )
{
    if ( img.type( ) == CV_8UC1 )
        integral = cv::Mat( img.rows, img.cols, CV_32SC1 );

    int nr = img.rows, nc = img.cols;
    int index_col, index_row;

    const uchar* pImg = img.ptr< uchar >( 0 );
    int* pIntegral    = integral.ptr< int >( 0 );

    for ( index_row = 0; index_row < nr; ++index_row )
    {
        pImg      = img.ptr< uchar >( index_row );
        pIntegral = integral.ptr< int >( index_row );

        pIntegral[0] = pImg[0];

        for ( index_col = 1; index_col < nc; ++index_col )
        {
            pIntegral[index_col] = pIntegral[index_col - 1] + pImg[index_col];
        }
    }
}

void
sumPixelRow2( const cv::Mat& img, cv::Mat& integral )
{
    if ( img.type( ) == CV_8UC1 )
        integral = cv::Mat( img.rows, img.cols, CV_32SC1 );

    int nr = img.rows, nc = img.cols;
    int index_col, index_row;

    const uchar* pImg = img.ptr< uchar >( 0 );
    int* pIntegral    = integral.ptr< int >( 0 );

    pIntegral[0] = pImg[0];
    for ( index_col = 1; index_col < nc; ++index_col )
    {
        pIntegral[index_col] = pIntegral[index_col - 1] + pImg[index_col];
    }

    for ( index_row = 1; index_row < nr; ++index_row )
    {
        pImg      = img.ptr< uchar >( index_row );
        pIntegral = integral.ptr< int >( index_row );
        for ( index_col = 0; index_col < nc; ++index_col )
        {
            pIntegral[index_col] = pIntegral[index_col - 1] + pImg[index_col];
        }
    }
}

void
test1( )
{
    Mat img;
    Mat img1 = imread( "/home/gao/IMG_1.png", CV_LOAD_IMAGE_GRAYSCALE );

    sys_utils::tic::TicTocPart time;

    for ( int i = 0; i < 100; i++ )
        sumPixelRow( img1, img );

    std::cout << "sumPixelRow cost " << time.toc( ) << " ms\n";

    cv::Mat img2;
    normalize( img, img2, 0, 255, CV_MINMAX );
    Mat imageIntegralNorm;
    convertScaleAbs( img2, imageIntegralNorm );

    showImg( "src1", img1 );
    showImg( "dst1", img );
    showImg( "dst11", imageIntegralNorm );
}

void
test2( )
{
    Mat img;
    Mat img1 = imread( "/home/gao/IMG_1.png", CV_LOAD_IMAGE_GRAYSCALE );

    sys_utils::tic::TicTocPart time;

    for ( int i = 0; i < 100; i++ )
        integral( img1, img, img1.type( ) );

    std::cout << "sumPixelRow cost " << time.toc( ) << " ms\n";

    cv::Mat img2;
    normalize( img, img2, 0, 255, CV_MINMAX );
    Mat imageIntegralNorm;
    convertScaleAbs( img2, imageIntegralNorm );

    showImg( "src2", img1 );
    showImg( "dst2", img );
    showImg( "dst22", imageIntegralNorm );
}

int
main( )
{
    test1( );
    test2( );

    waitKey( 0 );

    return 0;
}
