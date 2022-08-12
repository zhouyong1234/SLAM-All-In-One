#ifndef FLOAT_EQUAL_HPP
#define FLOAT_EQUAL_HPP

#include <cmath>
#include <iostream>

namespace sys_utils
{

namespace equal
{

inline bool
float_equal( const float a, const float b )
{
    if ( std::abs( a - b ) < 1e-6 )
        return true;
    else
        return false;
}

inline bool
double_equal( const double a, const double b )
{
    if ( std::abs( a - b ) < 1e-6 )
        return true;
    else
        return false;
}
}
}
#endif // FLOAT_EQUAL_HPP
