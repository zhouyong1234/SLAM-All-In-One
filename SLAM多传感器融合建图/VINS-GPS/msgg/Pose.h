#ifndef GEOMETRY_MSGS_MESSAGE_POSE_H
#define GEOMETRY_MSGS_MESSAGE_POSE_H


#include <string>
#include <vector>
#include <map>

#include "Point.h"
#include "Quaternion.h"

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Pose_
{
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
    : position()
    , orientation()  {
    }
  Pose_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , orientation(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;




  typedef boost::shared_ptr< ::geometry_msgs::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::geometry_msgs::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::geometry_msgs::Pose > PosePtr;
typedef boost::shared_ptr< ::geometry_msgs::Pose const> PoseConstPtr;
}
#endif
