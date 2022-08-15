//
// Created by JinTian on 18/03/2018.
//

#include "mono_camera.h"

MonoCamera::MonoCamera() {

}

MonoCamera::MonoCamera(int width, int height, double fx, double fy,
                       double cx, double cy, double k1, double k2,
                       double p1, double p2, double k3) {
    width_ = width;
    height_ = height;
    fx_= fx;
    fy_= fy;

    cx_ =cx;
    cy_ = cy;

    d_[0] = k1;
    d_[1] = k2;
    d_[2] = k3;
    d_[3] = p1;
    d_[4] = p2;

}
