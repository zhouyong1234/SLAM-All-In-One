#ifndef EIGEN_FILE_IO_HPP
#define EIGEN_FILE_IO_HPP

#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace sys_utils
{

namespace io
{

template< class Matrix >
void
writeMatrixToBinary( const std::string filename, const Matrix& matrix )
{
    std::ofstream out( filename, std::ios::out | std::ios::binary | std::ios::trunc );
    typename Matrix::Index rows = matrix.rows( ), cols = matrix.cols( );
    out.write( ( char* )( &rows ), sizeof( typename Matrix::Index ) );
    out.write( ( char* )( &cols ), sizeof( typename Matrix::Index ) );
    out.write( ( char* )matrix.data( ), rows * cols * sizeof( typename Matrix::Scalar ) );
    out.close( );
}

template< class Matrix >
void
parseMatrixFromBinary( const std::string filename, Matrix& matrix )
{
    std::ifstream in( filename, std::ios::in | std::ios::binary );
    while ( in.peek( ) != EOF )
    {
        typename Matrix::Index rows = 0, cols = 0;
        in.read( ( char* )( &rows ), sizeof( typename Matrix::Index ) );
        in.read( ( char* )( &cols ), sizeof( typename Matrix::Index ) );
        matrix.resize( rows, cols );
        in.read( ( char* )matrix.data( ), rows * cols * sizeof( typename Matrix::Scalar ) );
    }
    in.close( );
}
}
}

#endif // EIGEN_FILE_IO_HPP
