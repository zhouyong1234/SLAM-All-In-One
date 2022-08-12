#ifndef CV_TYPE_HPP
#define CV_TYPE_HPP

#include <opencv2/core.hpp>

namespace cv
{

#define CV_8UC12 CV_MAKETYPE( CV_8U, 12 )

#define CV_8SC12 CV_MAKETYPE( CV_8S, 12 )
#define CV_8SC16 CV_MAKETYPE( CV_8S, 16 )
#define CV_8SC24 CV_MAKETYPE( CV_8S, 24 )
#define CV_8SC32 CV_MAKETYPE( CV_8S, 32 )

#define CV_16SC12 CV_MAKETYPE( CV_8S, 12 )

#define CV_32SC12 CV_MAKETYPE( CV_32S, 12 )
#define CV_32SC24 CV_MAKETYPE( CV_32S, 24 )

#define CV_32FC5 CV_MAKETYPE( CV_32F, 5 )

typedef Vec< uchar, 12 > Vec12b;

typedef Vec< short, 12 > Vec12s;
typedef Vec< short, 16 > Vec16s;
typedef Vec< short, 24 > Vec24s;
typedef Vec< short, 32 > Vec32s;

typedef Vec< char, 3 > Vec3c;
typedef Vec< char, 4 > Vec4c;

typedef Vec< char, 12 > Vec12c;
typedef Vec< char, 16 > Vec16c;
typedef Vec< char, 24 > Vec24c;
typedef Vec< char, 32 > Vec32c;

typedef Vec< int, 12 > Vec12i;
typedef Vec< int, 16 > Vec16i;
typedef Vec< int, 24 > Vec24i;
typedef Vec< int, 32 > Vec32i;

typedef Vec< float, 5 > Vec5f;
}

#endif // CV_TYPE_HPP
