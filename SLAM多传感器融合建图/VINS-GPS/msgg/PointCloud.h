#ifndef SENSOR_MSGS_MESSAGE_POINTCLOUD_H
#define SENSOR_MSGS_MESSAGE_POINTCLOUD_H


#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>


#include "Header.h"
#include "Point32.h"
#include "ChannelFloat32.h"

namespace sensor_msgs
{
template <class ContainerAllocator>
struct PointCloud_
{
  typedef PointCloud_<ContainerAllocator> Type;

  PointCloud_()
    : header()
    , points()
    , channels()  {
    }
  PointCloud_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , points(_alloc)
    , channels(_alloc)  {
  (void)_alloc;
    }



   typedef  std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< geometry_msgs::Point32_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef std::vector< sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename ContainerAllocator::template rebind< sensor_msgs::ChannelFloat32_<ContainerAllocator> >::other >  _channels_type;
  _channels_type channels;




  typedef boost::shared_ptr< sensor_msgs::PointCloud_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< sensor_msgs::PointCloud_<ContainerAllocator> const> ConstPtr;

}; // struct PointCloud_

typedef sensor_msgs::PointCloud_<std::allocator<void> > PointCloud;

typedef boost::shared_ptr< sensor_msgs::PointCloud > PointCloudPtr;
typedef boost::shared_ptr< sensor_msgs::PointCloud const> PointCloudConstPtr;
}
#endif

