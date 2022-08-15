#ifndef GEOMETRY_MSGS_MESSAGE_POSESTAMPED_H
#define GEOMETRY_MSGS_MESSAGE_POSESTAMPED_H


#include <string>
#include <vector>
#include <map>


#include "Header.h"
#include "Pose.h"

namespace geometry_msgs
{
template <class ContainerAllocator>
struct PoseStamped_
{
  typedef PoseStamped_<ContainerAllocator> Type;

  PoseStamped_()
    : header()
    , pose()  {
    }
  PoseStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;




  typedef boost::shared_ptr< ::geometry_msgs::PoseStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::PoseStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PoseStamped_

typedef ::geometry_msgs::PoseStamped_<std::allocator<void> > PoseStamped;

typedef boost::shared_ptr< ::geometry_msgs::PoseStamped > PoseStampedPtr;
typedef boost::shared_ptr< ::geometry_msgs::PoseStamped const> PoseStampedConstPtr;
}
#endif
