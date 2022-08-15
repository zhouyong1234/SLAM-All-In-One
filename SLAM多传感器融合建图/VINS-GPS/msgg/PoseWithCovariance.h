
#ifndef GEOMETRY_MSGS_MESSAGE_POSEWITHCOVARIANCE_H
#define GEOMETRY_MSGS_MESSAGE_POSEWITHCOVARIANCE_H


#include <string>
#include <vector>
#include <map>
#include <boost/array.hpp>


#include "Pose.h"

namespace geometry_msgs
{
template <class ContainerAllocator>
struct PoseWithCovariance_
{
  typedef PoseWithCovariance_<ContainerAllocator> Type;

  PoseWithCovariance_()
    : pose()
    , covariance()  {
      covariance.assign(0.0);
  }
  PoseWithCovariance_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef boost::array<double, 36>  _covariance_type;
  _covariance_type covariance;




  typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct PoseWithCovariance_

typedef ::geometry_msgs::PoseWithCovariance_<std::allocator<void> > PoseWithCovariance;

typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance > PoseWithCovariancePtr;
typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance const> PoseWithCovarianceConstPtr;
}
#endif
