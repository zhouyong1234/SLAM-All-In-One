#ifndef SENSOR_MSGS_MESSAGE_IMU_H
#define SENSOR_MSGS_MESSAGE_IMU_H


#include <string>
#include <vector>
#include <map>
#include <boost/array.hpp>

#include "Time.h"
#include "Quaternion.h"
#include "Vector3.h"
#include "Header.h"


namespace sensor_msgs
{
template <class ContainerAllocator>
struct Imu_
{
  typedef Imu_<ContainerAllocator> Type;

  Imu_()
    : header()
    , orientation()
    , orientation_covariance()
    , angular_velocity()
    , angular_velocity_covariance()
    , linear_acceleration()
    , linear_acceleration_covariance()  {
      orientation_covariance.assign(0.0);

      angular_velocity_covariance.assign(0.0);

      linear_acceleration_covariance.assign(0.0);
  }
  Imu_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , orientation(_alloc)
    , orientation_covariance()
    , angular_velocity(_alloc)
    , angular_velocity_covariance()
    , linear_acceleration(_alloc)
    , linear_acceleration_covariance()  {
  (void)_alloc;
      orientation_covariance.assign(0.0);

      angular_velocity_covariance.assign(0.0);

      linear_acceleration_covariance.assign(0.0);
  }



   typedef  std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;

   typedef boost::array<double, 9>  _orientation_covariance_type;
  _orientation_covariance_type orientation_covariance;

   typedef  geometry_msgs::Vector3_<ContainerAllocator>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;

   typedef boost::array<double, 9>  _angular_velocity_covariance_type;
  _angular_velocity_covariance_type angular_velocity_covariance;

   typedef  geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef boost::array<double, 9>  _linear_acceleration_covariance_type;
  _linear_acceleration_covariance_type linear_acceleration_covariance;




  typedef boost::shared_ptr< sensor_msgs::Imu_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< sensor_msgs::Imu_<ContainerAllocator> const> ConstPtr;

}; // struct Imu_

typedef sensor_msgs::Imu_<std::allocator<void> > Imu;

typedef boost::shared_ptr< sensor_msgs::Imu > ImuPtr;
typedef boost::shared_ptr< sensor_msgs::Imu const> ImuConstPtr;
}
#endif

