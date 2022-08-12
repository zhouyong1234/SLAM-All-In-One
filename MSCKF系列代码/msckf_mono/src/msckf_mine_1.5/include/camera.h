#ifndef CAMERA_H
#define CAMERA_H
#include "common_include.h"

namespace MSCKF_MINE
{


class Camera
{
public:

    Camera();
    ~Camera();


    double      fx;
    double      fy;
    double      cx;
    double      cy;
    double      k1;
    double      k2;
    double      p1;
    double      p2;
    double      sigma_img;
    Mat         K;    /*Camera intrinsics*/
    Mat         TBS;  /*Sensor extrinsics wrt. the body-frame.*/
    Mat         D;    /*distortion_coefficients*/


    Mat getK();
    Mat getTBS();
    Mat getD();


};




}

#endif // CAMERA_H
