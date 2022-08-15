
#ifndef STD_MSGS_MESSAGE_FLOAT32_H
#define STD_MSGS_MESSAGE_FLOAT32_H


#include <string>
#include <vector>
#include <map>




namespace std_msgs
{
template <class ContainerAllocator>
struct Float32_
{
  typedef Float32_<ContainerAllocator> Type;

  Float32_()
    : data(0.0)  {
    }
  Float32_(const ContainerAllocator& _alloc)
    : data(0.0)  {
  (void)_alloc;
    }



   typedef float _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::std_msgs::Float32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::std_msgs::Float32_<ContainerAllocator> const> ConstPtr;

}; // struct Float32_
}
#endif

