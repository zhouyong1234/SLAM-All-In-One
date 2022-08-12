#ifndef UTILS_H
#define UTILS_H

namespace math_utils
{

template< typename T >
inline T
max_in_three( T a, T b, T c )
{
    return a >= b ? ( a >= c ? a : c ) : ( b >= c ? b : c );
}

template< typename T >
inline T
max_index_in_three( T a, T b, T c )
{
    return a >= b ? ( a >= c ? T( 0 ) : T( 2 ) ) : ( b >= c ? T( 1 ) : T( 2 ) );
}
}
#endif // UTILS_H
