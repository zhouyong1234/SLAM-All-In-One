#ifndef NAV_MSGS_MESSAGE_ODOMETRY_H
#define NAV_MSGS_MESSAGE_ODOMETRY_H


#include <string>
#include <vector>
#include <map>

#include "Header.h"
#include "PoseWithCovariance.h"
#include "TwistWithCovariance.h"

namespace nav_msgs
{
template <class ContainerAllocator>
struct Odometry_
{
  typedef Odometry_<ContainerAllocator> Type;

  Odometry_()
    : header()
    , child_frame_id()
    , pose()
    , twist()  {
    }
  Odometry_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , child_frame_id(_alloc)
    , pose(_alloc)
    , twist(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _child_frame_id_type;
  _child_frame_id_type child_frame_id;

   typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::TwistWithCovariance_<ContainerAllocator>  _twist_type;
  _twist_type twist;




  typedef boost::shared_ptr< ::nav_msgs::Odometry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav_msgs::Odometry_<ContainerAllocator> const> ConstPtr;

}; // struct Odometry_

typedef ::nav_msgs::Odometry_<std::allocator<void> > Odometry;

typedef boost::shared_ptr< ::nav_msgs::Odometry > OdometryPtr;
typedef boost::shared_ptr< ::nav_msgs::Odometry const> OdometryConstPtr;
}
#endif
