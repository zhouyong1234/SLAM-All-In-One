#ifndef GEOMETRY_MSGS_MESSAGE_TWIST_H
#define GEOMETRY_MSGS_MESSAGE_TWIST_H


#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>



#include "Vector3.h"


namespace geometry_msgs
{
template <class ContainerAllocator>
struct Twist_
{
  typedef Twist_<ContainerAllocator> Type;

  Twist_()
    : linear()
    , angular()  {
    }
  Twist_(const ContainerAllocator& _alloc)
    : linear(_alloc)
    , angular(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_type;
  _linear_type linear;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_type;
  _angular_type angular;




  typedef boost::shared_ptr< ::geometry_msgs::Twist_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Twist_<ContainerAllocator> const> ConstPtr;

}; // struct Twist_

typedef ::geometry_msgs::Twist_<std::allocator<void> > Twist;

typedef boost::shared_ptr< ::geometry_msgs::Twist > TwistPtr;
typedef boost::shared_ptr< ::geometry_msgs::Twist const> TwistConstPtr;

// constants requiring out of line definition

} 
#endif
