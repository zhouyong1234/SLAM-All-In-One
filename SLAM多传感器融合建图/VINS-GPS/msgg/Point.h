#ifndef GEOMETRY_MSGS_MESSAGE_POINT_H
#define GEOMETRY_MSGS_MESSAGE_POINT_H


#include <string>
#include <vector>
#include <map>

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Point_
{
  typedef Point_<ContainerAllocator> Type;

  Point_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Point_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::geometry_msgs::Point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Point_<ContainerAllocator> const> ConstPtr;

}; // struct Point_

typedef ::geometry_msgs::Point_<std::allocator<void> > Point;

typedef boost::shared_ptr< ::geometry_msgs::Point > PointPtr;
typedef boost::shared_ptr< ::geometry_msgs::Point const> PointConstPtr;
}
#endif
