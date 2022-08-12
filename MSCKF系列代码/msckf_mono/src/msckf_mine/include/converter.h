#ifndef CONVERTER_H
#define CONVERTER_H
#include<opencv2/core/core.hpp>
#include<Eigen/Dense>
#include <map>

using namespace std;

namespace MSCKF_MINE
{
class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvMat(const Eigen::Quaterniond &q);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix4d toMatrix4d(const cv::Mat &cvMat4);

    //static std::vector<float> toQuaternion(const cv::Mat &M);
    static Eigen::Quaterniond toQuaternion(const cv::Mat &M);

    //static Sophus::SE3 toSE3(const cv::Mat &cvT);
    static cv::Mat toCvMat(const Eigen::Vector3d &m, int flag);
    static map<int,int> swapMatchesId(const map<int,int>& matchesId);
    static Eigen::Vector4d toWxyz(Eigen::Vector4d &vec);


};


}

#endif // CONVERTER_H
