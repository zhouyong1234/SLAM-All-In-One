#ifndef CVMAT_FILE_IO_HPP
#define CVMAT_FILE_IO_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace sys_utils
{

namespace io
{

//                 | C1  C2   C3	C4
// CV_8U  | uchar  | 0   8    16	24 |  -0 %8
// CV_8S  | char   | 1   9    17	25 |  -1 %8
// CV_16U |        | 2   10   18	26 |  -2 %8
// CV_16S | short  | 3   11   19	27 |  -3 %8
// CV_32S | int    | 4   12   20	28 |  -4 %8
// CV_32F | float  | 5   13   21	29 |  -5 %8
// CV_64F | double | 6   14   22	30 |  -6 %8

inline void
writeMatrixToBinary( const std::string filename, const cv::Mat& matrix )
{
    std::ofstream out( filename, std::ios::out | std::ios::binary | std::ios::trunc );
    int rows = matrix.rows, cols = matrix.cols, step = matrix.step, type = matrix.type( );

    out.write( ( char* )( &rows ), sizeof( int ) );
    out.write( ( char* )( &cols ), sizeof( int ) );
    out.write( ( char* )( &step ), sizeof( int ) );
    out.write( ( char* )( &type ), sizeof( int ) );

    // std::cout << " rows " << rows << "\n";
    // std::cout << " cols " << cols << "\n";
    // std::cout << " step " << step << "\n";
    // std::cout << " type " << type << "\n";

    out.write( ( char* )matrix.data, rows * step * sizeof( uchar ) );

    out.close( );
}

inline void
parseMatrixFromBinary( const std::string filename, cv::Mat& matrix )
{
    std::ifstream in( filename, std::ios::in | std::ios::binary );
    while ( in.peek( ) != EOF )
    {
        int rows = 0, cols = 0, step = 0, type = 0;

        in.read( ( char* )( &rows ), sizeof( int ) );
        in.read( ( char* )( &cols ), sizeof( int ) );
        in.read( ( char* )( &step ), sizeof( int ) );
        in.read( ( char* )( &type ), sizeof( int ) );

        // std::cout << " rows " << rows << "\n";
        // std::cout << " cols " << cols << "\n";
        // std::cout << " step " << step << "\n";
        // std::cout << " type " << type << "\n";

        matrix = cv::Mat( rows, cols, type );

        in.read( ( char* )matrix.data, rows * step * sizeof( uchar ) );
    }

    in.close( );
}
}
}

#endif // CVMAT_FILE_IO_HPP
