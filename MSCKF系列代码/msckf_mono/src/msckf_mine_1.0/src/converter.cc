#include "converter.h"
#include "common_include.h"


namespace MSCKF_MINE
{
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_64F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_64F);
    for(int i=0;i<3;i++)
        cvMat.at<double>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d eigMat =  q.toRotationMatrix();
    cv::Mat R = toCvMat(eigMat);
    return R.clone();

}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    cv::Mat cvVectorCopy;
    if(cvVectorCopy.type() == 6)
    {
        cvVector.convertTo(cvVectorCopy,CV_64FC1);
        v << cvVectorCopy.at<double>(0), cvVectorCopy.at<double>(1), cvVectorCopy.at<double>(2);
    }
    else
        v << cvVector.at<double>(0), cvVector.at<double>(1), cvVector.at<double>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M = Eigen::Matrix3d::Identity();
    cv::Mat cvMat3Copy;

    cvMat3.convertTo(cvMat3Copy,CV_64FC1);
       M << cvMat3Copy.at<double>(0,0), cvMat3Copy.at<double>(0,1), cvMat3Copy.at<double>(0,2),
            cvMat3Copy.at<double>(1,0), cvMat3Copy.at<double>(1,1), cvMat3Copy.at<double>(1,2),
            cvMat3Copy.at<double>(2,0), cvMat3Copy.at<double>(2,1), cvMat3Copy.at<double>(2,2);
    return M;
}

Eigen::Matrix4d Converter::toMatrix4d(const Mat &cvMat4)
{
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    cv::Mat cvMat4Copy;

    cvMat4.convertTo(cvMat4Copy,CV_64FC1);
       M << cvMat4Copy.at<double>(0,0), cvMat4Copy.at<double>(0,1), cvMat4Copy.at<double>(0,2), cvMat4Copy.at<double>(0,3),
            cvMat4Copy.at<double>(1,0), cvMat4Copy.at<double>(1,1), cvMat4Copy.at<double>(1,2), cvMat4Copy.at<double>(1,3),
            cvMat4Copy.at<double>(2,0), cvMat4Copy.at<double>(2,1), cvMat4Copy.at<double>(2,2), cvMat4Copy.at<double>(2,3),
            cvMat4Copy.at<double>(3,0), cvMat4Copy.at<double>(3,1), cvMat4Copy.at<double>(3,2), cvMat4Copy.at<double>(3,3);
    return M;
}

Eigen::Quaterniond Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);
    return q;
}


cv::Mat Converter::toCvMat(const Eigen::Vector3d &m, int flag = 1)
{
    if(flag)
    {
        cv::Mat T = cv::Mat::zeros(3,1,CV_64FC1);
        for(int i = 0; i < 3; i++)
        {
            T.at<double>(i,0) = m(i,0);
        }
        return T.clone();
    }
    else
    {
        cv::Mat T = cv::Mat::zeros(3,1,CV_32FC1);
        for(int i = 0; i < 3; i++)
        {
            T.at<float>(i,0) = m(i,0);
        }
        return T.clone();
    }

}

map<int,int> Converter::swapMatchesId(const map<int, int> &matchesId)
{
    map<int,int> matches;
    for(map<int,int>::const_iterator iter = matchesId.begin(); iter!=matchesId.end(); iter++ )
    {
        matches[iter->second] = iter->first;  /*feedframe curr_frame*/
    }

    return matches;
}

Eigen::Vector4d Converter::toWxyz(Eigen::Vector4d &vec)
{
    Eigen::Vector4d v(vec(3), vec(0), vec(1), vec(2));
    return v;
}


}


