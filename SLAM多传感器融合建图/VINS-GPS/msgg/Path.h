#ifndef NAV_MSGS_MESSAGE_PATH_H
#define NAV_MSGS_MESSAGE_PATH_H


#include <string>
#include <vector>
#include <map>


#include "Header.h"
#include "PoseStamped.h"

namespace nav_msgs
{
template <class ContainerAllocator>
struct Path_
{
  typedef Path_<ContainerAllocator> Type;

  Path_()
    : header()
    , poses()  {
    }
  Path_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , poses(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _poses_type;
  _poses_type poses;




  typedef boost::shared_ptr< ::nav_msgs::Path_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav_msgs::Path_<ContainerAllocator> const> ConstPtr;

}; // struct Path_

typedef ::nav_msgs::Path_<std::allocator<void> > Path;

typedef boost::shared_ptr< ::nav_msgs::Path > PathPtr;
typedef boost::shared_ptr< ::nav_msgs::Path const> PathConstPtr;
}
#endif
