#ifndef SENSOR_MSGS_MESSAGE_CHANNELFLOAT32_H
#define SENSOR_MSGS_MESSAGE_CHANNELFLOAT32_H


#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>


namespace sensor_msgs
{
template <class ContainerAllocator>
struct ChannelFloat32_
{
  typedef ChannelFloat32_<ContainerAllocator> Type;

  ChannelFloat32_()
    : name()
    , values()  {
    }
  ChannelFloat32_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , values(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _values_type;
  _values_type values;




  typedef boost::shared_ptr< sensor_msgs::ChannelFloat32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< sensor_msgs::ChannelFloat32_<ContainerAllocator> const> ConstPtr;

}; // struct ChannelFloat32_

typedef sensor_msgs::ChannelFloat32_<std::allocator<void> > ChannelFloat32;

typedef boost::shared_ptr< sensor_msgs::ChannelFloat32 > ChannelFloat32Ptr;
typedef boost::shared_ptr< sensor_msgs::ChannelFloat32 const> ChannelFloat32ConstPtr;
}

#endif

