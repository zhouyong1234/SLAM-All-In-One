//
// Created by JinTian on 18/03/2018.
//

#ifndef MONO_ODOMETRY_MONO_CAMERA_H
#define MONO_ODOMETRY_MONO_CAMERA_H


class MonoCamera{
public:
    MonoCamera();

    MonoCamera(int width, int height,
                  double fx, double fy, double cx, double cy,
                  double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

    inline int width() const { return width_;}
    inline int height() const { return height_;}

    inline double fx() const { return fx_;}
    inline double fy() const { return fy_;}
    inline double cx() const { return cx_;}
    inline double cy() const { return cy_;}

    // distortion params, most time we should ignore this?
    inline double k1() const { return d_[0]; };
    inline double k2() const { return d_[1]; };
    inline double p1() const { return d_[2]; };
    inline double p2() const { return d_[3]; };
    inline double k3() const { return d_[4]; };

protected:

    // camera params
    double fx_, fy_;
    int width_, height_;
    double cx_, cy_;

    // distortion
    bool distortion_;
    double d_[5];


};


#endif //MONO_ODOMETRY_MONO_CAMERA_H
