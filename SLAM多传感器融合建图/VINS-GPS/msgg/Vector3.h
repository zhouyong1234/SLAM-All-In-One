#ifndef GEOMETRY_MSGS_MESSAGE_VECTOR3_H
#define GEOMETRY_MSGS_MESSAGE_VECTOR3_H


#include <string>
#include <vector>
#include <map>


namespace geometry_msgs
{
template <class ContainerAllocator>
struct Vector3_
{
  typedef Vector3_<ContainerAllocator> Type;

  Vector3_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Vector3_(const ContainerAllocator& _alloc)
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




  typedef boost::shared_ptr< geometry_msgs::Vector3_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< geometry_msgs::Vector3_<ContainerAllocator> const> ConstPtr;

}; // struct Vector3_

typedef geometry_msgs::Vector3_<std::allocator<void> > Vector3;

typedef boost::shared_ptr< geometry_msgs::Vector3 > Vector3Ptr;
typedef boost::shared_ptr< geometry_msgs::Vector3 const> Vector3ConstPtr;
}
#endif

