#pragma once

#include <Eigen/Core>

using namespace Eigen;

namespace geo_utils {
const double a = 6378137;
const double e2 = 0.00669437999013;
const double pi = 3.14159265358979324;

const int maxtypeofobs = 15;
const int maxsatnume = 50;

const double c = 2.99792458E8; //m/s
const double GM = 3.986005E14; //m^3/s^2
const double omega_e = 0.000072921151467;//rad/s

const double ura_eph[]={         /* ura values (ref [3] 20.3.3.3.1.1) */
                                 2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
                                 3072.0,6144.0,0.0
                       };

const double lamda0 = c/(1575.42E6);  //GPSÉÏL1ÔØ²š²š³€£šm£©
const double lamda1 = c/120.0/10.23E6;  //GPSÉÏL2ÔØ²š²š³€£šm£©

typedef Vector3d Geodetic;
typedef Vector3d Cartesian;
typedef Vector3d Topocentric;
typedef Vector3d Topopolar;

class CoordTrans{
public:
    static Geodetic Cart2Geod(const Cartesian &X); //µÑ¿š¶û×ø±êÏµ->ŽóµØ×ø±êÏµ
    static Cartesian Geod2Cart(const Geodetic &G); //ŽóµØ×ø±êÏµ->µÑ¿š¶û×ø±êÏµ
    static Matrix3d getRne(const Geodetic &x);
};

Geodetic CoordTrans::Cart2Geod(const Cartesian &x)
{
    Geodetic ans;
    double B1, B2, N, W;
    double X=x.x(), Y=x.y(), Z=x.z();
    double L, B, H;
    L = atan( Y / X);
    L = L / pi * 180;
    if (X<0 && Y>0) L = L + 180;
    if (X < 0 && Y<0) L = L - 180;
    B1 = atan( Z / sqrt(X*X + Y*Y));
    N = a / sqrt(1 - e2*sin(B1)*sin(B1));
    B2 = atan((Z + N*e2*sin(B1)) / sqrt(X*X + Y*Y));
    /**************µüŽú·šÇóŽóµØÎ³¶È	************************/
    while (abs(B1 - B2) >(0.001 / 3600 * pi / 180))
    {
        B1 = B2;
        N = a / sqrt(1 - e2*sin(B1)*sin(B1));
        B2 = atan((Z + N*e2*sin(B1)) / sqrt(X*X + Y*Y));
    }
    B = B2 / pi * 180;
    H = Z / sin(B2) - N*(1 - e2);
    return (Vector3d(B,L,H));
}


/**************¶šÒåŽóµØ×ø±ê×ªµÑ¿š¶û×ø±êº¯Êý***********************************/
/*
   º¯ÊýÃû£ºGeod2Cart()
   ÊäÈë£ºŽóµØ×ø±êG
   Êä³ö£ºµÑ¿š¶û×ø±ê
*/
Cartesian CoordTrans::Geod2Cart(const Geodetic &x)
{
    Cartesian ans;
    double N, L, B, H;
    double X, Y, Z;
    B = x.x() / 180 * pi;
    L = x.y() / 180 * pi;
    H = x.z();
    N = a / sqrt(1 - e2*sin(B)*sin(B));
    X = (N + H)*cos(B)*cos(L);
    Y = (N + H)*cos(B)*sin(L);
    Z = (N*(1 - e2) + H)*sin(B);
    return Vector3d(X,Y,Z);
}

Matrix3d CoordTrans::getRne(const Geodetic &x)
{
    double B = x.x()/180*pi;
    double L = x.y()/180*pi;
    Matrix3d R;
    double sinB = sin(B);
    double cosB = cos(B);
    double sinL = sin(L);
    double cosL = cos(L);
    R(0, 0) = -sinL;			R(0, 1) = cosL;				R(0, 2) = 0;//E
    R(1, 0) = -cosL*sinB;		R(1, 1) = -sinL*sinB;			R(1, 2) = cosB;//N
    R(2, 0) = cosL*cosB;		R(2, 1) = sinL*cosB;			R(2, 2) = sinB;//U
    return R;
}


}
