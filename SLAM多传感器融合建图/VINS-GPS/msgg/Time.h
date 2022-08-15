/*********************************************************************
 *all edit by solomon
 *solomon.he@zhaoxin.com
 *********************************************************************/

#ifndef ROS_TIME_H_INCLUDED
#define ROS_TIME_H_INCLUDED

/*********************************************************************
 ** Pragmas
 *********************************************************************/
 
/*********************************************************************
 ** Headers
 *********************************************************************/

//#include <ros/platform.h>
#include <iostream>
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include <sys/time.h>


/*********************************************************************
 ** Cross Platform Headers
 *********************************************************************/

namespace  ros //DataStucture
{
 /*********************************************************************
 ** Functions
 *********************************************************************/

     inline void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
     {
       uint64_t nsec_part = nsec % 1000000000UL;
       uint64_t sec_part = nsec / 1000000000UL;
     
       if (sec_part > UINT_MAX)
         throw std::runtime_error("Time is out of dual 32-bit range");
     
       sec += sec_part;
       nsec = nsec_part;
     }
     
     inline void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
     {
       uint64_t sec64 = sec;
       uint64_t nsec64 = nsec;
     
       normalizeSecNSec(sec64, nsec64);
     
       sec = (uint32_t)sec64;
       nsec = (uint32_t)nsec64;
    }
 
   inline void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
   {
    int64_t nsec_part = nsec;
    int64_t sec_part = sec;
 
    while (nsec_part >= 1000000000L)
    {
      nsec_part -= 1000000000L;
      ++sec_part;
    }
     while (nsec_part < 0)
    {
      nsec_part += 1000000000L;
      --sec_part;
    }
    if (sec_part < 0 || sec_part > INT_MAX)
     throw std::runtime_error("Time is out of dual 32-bit range");
 
    sec = sec_part;
    nsec = nsec_part;
   }

  /*********************************************************************
   ** Time Classes
   *********************************************************************/

  /**
   * \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
   * This should not need to be used directly.
   */
  template<class T>
  class TimeBase
  {
  public:
    uint32_t sec, nsec;

    TimeBase() : sec(0), nsec(0) { }
    TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
    {
      normalizeSecNSec(sec, nsec);
    }
    explicit TimeBase(double t) { fromSec(t); }
    ~TimeBase() {}
  
    bool operator==(const T &rhs) const
    {return sec == rhs.sec && nsec == rhs.nsec;}
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const
    {
     if(sec > rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec > rhs.nsec)
       return true;
     return false;
    }
    bool operator<(const T &rhs) const
    {
     if(sec < rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec < rhs.nsec)
       return true;
     return false;
    }
    bool operator>=(const T &rhs) const
    {
     if(sec > rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec >= rhs.nsec)
       return true;
     return false;
    }
    bool operator<=(const T &rhs) const
     {
     if(sec < rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec <= rhs.nsec)
       return true;
     return false;
    }

    double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
    T& fromSec(double t) {
      sec = (uint32_t)floor(t);
      nsec = (uint32_t)boost::math::round((t-sec) * 1e9);
      // avoid rounding errors
      sec += (nsec / 1000000000ul);
      nsec %= 1000000000ul;
      return *static_cast<T*>(this);
    }

    uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
    T& fromNSec(uint64_t t);

    inline bool isZero() const { return sec == 0 && nsec == 0; }
    inline bool is_zero() const { return isZero(); }

  };

  /**
   * \brief Time representation.  May either represent wall clock time or ROS clock time.
   *
   * ros::TimeBase provides most of its functionality.
   */
  class  Time : public TimeBase<Time>
  {
  public:
    Time()
      : TimeBase<Time>()
    {}

    Time(uint32_t _sec, uint32_t _nsec)
      : TimeBase<Time>(_sec, _nsec)
    {}

    explicit Time(double t) { fromSec(t); }


  };
}
#endif
