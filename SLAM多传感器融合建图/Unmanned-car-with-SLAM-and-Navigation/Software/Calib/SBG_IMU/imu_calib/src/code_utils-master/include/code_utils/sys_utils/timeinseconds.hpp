#ifndef TIMEINSECONDS_HPP
#define TIMEINSECONDS_HPP

#include <iostream>
#include <time.h>

namespace sys_utils
{

inline unsigned long long
timeInMicroseconds( void )
{
    struct timespec tp;

    clock_gettime( CLOCK_REALTIME, &tp );

    return ( tp.tv_sec * 1000000 + tp.tv_nsec / 1000 );
}

inline double
timeInSeconds( void )
{
    struct timespec tp;

    clock_gettime( CLOCK_REALTIME, &tp );

    return ( static_cast< double >( tp.tv_sec ) + static_cast< double >( tp.tv_nsec ) / 1000000000.0 );
}
}
#endif // TIMEINSECONDS_HPP
