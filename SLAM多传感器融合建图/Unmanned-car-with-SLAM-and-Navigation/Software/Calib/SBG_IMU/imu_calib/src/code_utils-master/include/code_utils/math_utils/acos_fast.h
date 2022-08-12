#ifndef ACOS_FAST_H
#define ACOS_FAST_H

template< typename T >
T
acosFaster_acc( const T x ) const
{
    T a = 1.43 + 0.59 * x;
    a   = ( a + ( 2 + 2 * x ) / a ) / 2;
    T b = 1.65 - 1.41 * x;
    b   = ( b + ( 2 - 2 * x ) / b ) / 2;
    T c = 0.88 - 0.77 * x;
    c   = ( c + ( 2 - a ) / c ) / 2;
    return ( 8 * ( c + ( 2 - a ) / c ) - ( b + ( 2 - 2 * x ) / b ) ) / 6;
}

template< typename T >
T
acosFaster_sqrt( const T x ) const
{
    T a = sqrt( T( 2 ) + T( 2 ) * x );
    T b = sqrt( T( 2 ) - T( 2 ) * x );
    T c = sqrt( T( 2 ) - a );
    return T( 8 ) / T( 3 ) * c - b / T( 3 );
}

template< typename T >
T
acosFaster_linear( const T x ) const
{
    return ( T( 3.14159 ) - T( 1.57079 ) * x );
}

#endif // ACOS_FAST_H
