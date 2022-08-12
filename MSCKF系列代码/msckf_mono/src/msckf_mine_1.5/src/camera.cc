#include "common_include.h"
#include "camera.h"
#include "config.h"
#include <sstream>

namespace MSCKF_MINE
{

Camera::Camera()
{
    fx           = Config::get<double>("Camera.fx");
    fy           = Config::get<double>("Camera.fy");
    cx           = Config::get<double>("Camera.cx");
    cy           = Config::get<double>("Camera.cy");
    k1           = Config::get<double>("Camera.k1");
    k2           = Config::get<double>("Camera.k2");
    p1           = Config::get<double>("Camera.p1");
    p2           = Config::get<double>("Camera.p2");
    sigma_img    = Config::get<double>("sigma_img");

    /*K*/
    Mat k = Mat::eye(3, 3, CV_64F);
    k.at<double>(0,0) = fx; k.at<double>(0,2) = cx;
    k.at<double>(1,1) = fy; k.at<double>(1,2) = cy;
    this->K = k.clone();

    /*D*/
    Mat d(5, 1, CV_64F);
    d.at<double>(0,0) = k1;
    d.at<double>(0,1) = k2;
    d.at<double>(0,2) = p1;
    d.at<double>(0,3) = p2;
    d.at<double>(0,4) = 0.0;

    this->D = d.clone();

    /*Tbs*/
    string Tbs = Config::get<string>("T_BS");
    //cout << BOLDCYAN"Camera.Tbs = " << Tbs << WHITE << endl;

    stringstream ss(Tbs);
    Mat tbs(4,4,CV_64F);
    for(int i = 0; i <4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            ss >> tbs.at<double>(i,j);
        }
    }

    TBS = tbs.clone();


}

Camera::~Camera()
{

}

Mat Camera::getK()
{
    return K.clone();
}

Mat Camera::getD()
{
    return D.clone();
}

Mat Camera::getTBS()
{
    return TBS.clone();
}




}
