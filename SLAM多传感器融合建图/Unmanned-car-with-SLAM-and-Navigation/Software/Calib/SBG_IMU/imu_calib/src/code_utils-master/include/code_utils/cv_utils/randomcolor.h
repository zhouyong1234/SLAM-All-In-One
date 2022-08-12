#ifndef RANDOMCOLOR_H
#define RANDOMCOLOR_H

#include <opencv2/core/core.hpp>

namespace cv
{

class RandomColor3
{
    public:
    RandomColor3( ) { randColor( ); }

    void randColor( )
    {
        color0 = rand( ) % 256;
        color1 = rand( ) % 256;
        color2 = rand( ) % 256;
    }
    cv::Scalar getColor( ) { return cv::Scalar( color0, color1, color2 ); }
    cv::Scalar getrandColor( )
    {
        randColor( );
        return cv::Scalar( color0, color1, color2 );
    }
    cv::Vec3b getColorVec( )
    {
        return cv::Vec3b( ( uchar )color0, ( uchar )color1, ( uchar )color2 );
    }

    private:
    int color0;
    int color1;
    int color2;
};

class RandomColor1
{
    public:
    RandomColor1( ) { randColor( ); }

    void randColor( ) { color0 = rand( ) % 256; }
    cv::Scalar getColor( ) { return cv::Scalar( color0 ); }
    cv::Scalar getrandColor( )
    {
        randColor( );
        return cv::Scalar( color0 );
    }

    private:
    int color0;
};
}

#endif // RANDOMCOLOR_H
