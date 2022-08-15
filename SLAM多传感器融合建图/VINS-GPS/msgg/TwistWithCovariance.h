#ifndef GEOMETRY_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H
#define GEOMETRY_MSGS_MESSAGE_TWISTWITHCOVARIANCE_H


#include <string>
#include <vector>
#include <map>
#include <boost/array.hpp>


#include "Twist.h"

namespace geometry_msgs
{
template <class ContainerAllocator>
struct TwistWithCovariance_
{
  typedef TwistWithCovariance_<ContainerAllocator> Type;

  TwistWithCovariance_()
    : twist()
    , covariance()  {
      covariance.assign(0.0);
  }
  TwistWithCovariance_(const ContainerAllocator& _alloc)
    : twist(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef boost::array<double, 36>  _covariance_type;
  _covariance_type covariance;




  typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct TwistWithCovariance_

typedef ::geometry_msgs::TwistWithCovariance_<std::allocator<void> > TwistWithCovariance;

typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance > TwistWithCovariancePtr;
typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance const> TwistWithCovarianceConstPtr;
}
#endif
