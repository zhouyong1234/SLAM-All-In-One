#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <glog/logging.h>
#include <chrono>
#include <queue>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                           (uint16_t, ring, ring) (float, time, time)
)

struct VelodynePointXYZIR
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                           (uint16_t, ring, ring)
)

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointXYZI>::Ptr PointCloudXYZIPtr;
typedef pcl::PointCloud<PointXYZI>::ConstPtr PointCloudXYZIConstPtr;

typedef VelodynePointXYZIRT PointVelodyne;
typedef pcl::PointCloud<PointVelodyne> PointCloudVelodyne;
typedef pcl::PointCloud<PointVelodyne>::Ptr PointCloudVelodynePtr;

typedef VelodynePointXYZIR PointVelodyne2;// 没有timestamp
typedef pcl::PointCloud<PointVelodyne2> PointCloudVelodyne2;
typedef pcl::PointCloud<PointVelodyne2>::Ptr PointCloudVelodynePtr2;

template <typename Point>
Eigen::Vector3d p2v(const Point& point){
    Eigen::Vector3d vector;
    vector.x() = point.x;
    vector.y() = point.y;
    vector.z() = point.z;
    return vector;
}

template <typename T>
pcl::PointXYZ v2p(Eigen::Matrix<T, 3, 1> v){
    pcl::PointXYZ point;
    point.x = v.x();
    point.y = v.y();
    point.z = v.z();
    return point;
}

template <typename Point>
Point pointTransform(Point point, Eigen::Affine3d trans){
    Eigen::Vector3d transformed_pose = (trans.rotation() * p2v(point) + trans.matrix().block<3, 1>(0, 3));
    Point result_point;
    result_point.x = transformed_pose.x();
    result_point.y = transformed_pose.y();
    result_point.z = transformed_pose.z();
    result_point.intensity = point.intensity;
    return result_point;
}

template <typename Point>
double pointDis(const Point& p1, const Point& p2){
    return (p2v(p1) - p2v(p2)).norm();
}

class Timer{
public:
    Timer(std::string name){
        name_ = name;
        start_timepoint_ = std::chrono::steady_clock::now();
    }

    void end(){
        auto end_time_point = std::chrono::steady_clock::now();
        LOG(INFO) << name_ << " elapsed " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time_point - start_timepoint_).count() << " ms.";
    }
private:
    std::chrono::steady_clock::time_point start_timepoint_;
    std::string name_;
};

#define ANG2RAD (1.0 / 180.0 * M_PI)
#define RAD2ANG (1 / M_PI* 180.0)

inline float findKthLargestValue(const std::vector<float>& values, int k){
    std::priority_queue<float, std::vector<float>, std::greater<float> > q;
    for(const float& value : values){
        if(q.size() < k){
            q.push(value);
        }else{
            if(value > q.top()){
                q.pop();
                q.push(value);
            }
        }
    }
    return q.top();
}


#endif

