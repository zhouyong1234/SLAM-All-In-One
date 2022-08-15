#ifndef GEOMETRY_MSGS_MESSAGE_POINT32_H
#define GEOMETRY_MSGS_MESSAGE_POINT32_H


#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>



namespace geometry_msgs
{
template <class ContainerAllocator>
struct Point32_
{
  typedef Point32_<ContainerAllocator> Type;

  Point32_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Point32_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> const> ConstPtr;

}; // struct Point32_

typedef ::geometry_msgs::Point32_<std::allocator<void> > Point32;

typedef boost::shared_ptr< ::geometry_msgs::Point32 > Point32Ptr;
typedef boost::shared_ptr< ::geometry_msgs::Point32 const> Point32ConstPtr;

}
#endif

