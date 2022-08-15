#ifndef STD_MSGS_MESSAGE_HEADER_H
#define STD_MSGS_MESSAGE_HEADER_H


#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include "Time.h"

namespace std_msgs
{
template <class ContainerAllocator>
struct Header_
{
  typedef Header_<ContainerAllocator> Type;

  Header_()
    : seq(0)
    , stamp()
    , frame_id()  {
    }
  Header_(const ContainerAllocator& _alloc)
    : seq(0)
    , stamp()
    , frame_id(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _seq_type;
  _seq_type seq;

   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _frame_id_type;
  _frame_id_type frame_id;




  typedef boost::shared_ptr< std_msgs::Header_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< std_msgs::Header_<ContainerAllocator> const> ConstPtr;

}; // struct Header_

typedef std_msgs::Header_<std::allocator<void> > Header;

typedef boost::shared_ptr< std_msgs::Header > HeaderPtr;
typedef boost::shared_ptr< std_msgs::Header const> HeaderConstPtr;

}
#endif
