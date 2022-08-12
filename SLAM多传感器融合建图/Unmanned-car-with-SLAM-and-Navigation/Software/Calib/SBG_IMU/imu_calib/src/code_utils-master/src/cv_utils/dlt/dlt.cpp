#include <code_utils/cv_utils/dlt/dlt.h>
#include <code_utils/cv_utils/pnp/utils.h>

cv::DLT::DLT( const std::vector< Eigen::Vector3d >& pts_2, const std::vector< Eigen::Vector3d >& pts_3 )
{
    m_num_points = ( int )pts_2.size( );

    if ( m_num_points != ( int )pts_3.size( ) )
        std::cout << " [ERROR] input points size Error!" << std::endl;

    M.resize( 2 * m_num_points, 12 );

    readPointsPlanar( pts_2, pts_3 );

    solveDLT( );
}

void
cv::DLT::readPointsPlanar( const std::vector< Eigen::Vector3d >& pts_2,
                           const std::vector< Eigen::Vector3d >& pts_3 )
{

    for ( int i = 0; i < m_num_points; i++ )
    {

        int max_i = math_utils::max_index_in_three( pts_2[i].x( ), pts_2[i].y( ), pts_2[i].z( ) );

        switch ( max_i )
        {
            case 0:
                M( 2 * i, 0 )      = -pts_3[i].x( ) * pts_2[i].y( ) / pts_2[i].x( );
                M( 2 * i, 1 )      = -pts_3[i].y( ) * pts_2[i].y( ) / pts_2[i].x( );
                M( 2 * i, 2 )      = -pts_3[i].z( ) * pts_2[i].y( ) / pts_2[i].x( );
                M( 2 * i, 3 )      = -pts_2[i].y( ) / pts_2[i].x( );
                M( 2 * i, 4 )      = pts_3[i].x( );
                M( 2 * i, 5 )      = pts_3[i].y( );
                M( 2 * i, 6 )      = pts_3[i].z( );
                M( 2 * i, 7 )      = 1;
                M( 2 * i, 8 )      = 0;
                M( 2 * i, 9 )      = 0;
                M( 2 * i, 10 )     = 0;
                M( 2 * i, 11 )     = 0;
                M( 2 * i + 1, 0 )  = -pts_3[i].x( ) * pts_2[i].z( ) / pts_2[i].x( );
                M( 2 * i + 1, 1 )  = -pts_3[i].y( ) * pts_2[i].z( ) / pts_2[i].x( );
                M( 2 * i + 1, 2 )  = -pts_3[i].z( ) * pts_2[i].z( ) / pts_2[i].x( );
                M( 2 * i + 1, 3 )  = -pts_2[i].z( ) / pts_2[i].x( );
                M( 2 * i + 1, 4 )  = 0;
                M( 2 * i + 1, 5 )  = 0;
                M( 2 * i + 1, 6 )  = 0;
                M( 2 * i + 1, 7 )  = 0;
                M( 2 * i + 1, 8 )  = pts_3[i].x( );
                M( 2 * i + 1, 9 )  = pts_3[i].y( );
                M( 2 * i + 1, 10 ) = pts_3[i].z( );
                M( 2 * i + 1, 11 ) = 1;
                break;
            case 1:
                M( 2 * i, 0 )      = pts_3[i].x( );
                M( 2 * i, 1 )      = pts_3[i].y( );
                M( 2 * i, 2 )      = pts_3[i].z( );
                M( 2 * i, 3 )      = 1;
                M( 2 * i, 4 )      = -pts_3[i].x( ) * pts_2[i].x( ) / pts_2[i].y( );
                M( 2 * i, 5 )      = -pts_3[i].y( ) * pts_2[i].x( ) / pts_2[i].y( );
                M( 2 * i, 6 )      = -pts_3[i].z( ) * pts_2[i].x( ) / pts_2[i].y( );
                M( 2 * i, 7 )      = -pts_2[i].x( ) / pts_2[i].y( );
                M( 2 * i, 8 )      = 0;
                M( 2 * i, 9 )      = 0;
                M( 2 * i, 10 )     = 0;
                M( 2 * i, 11 )     = 0;
                M( 2 * i + 1, 0 )  = 0;
                M( 2 * i + 1, 1 )  = 0;
                M( 2 * i + 1, 2 )  = 0;
                M( 2 * i + 1, 3 )  = 0;
                M( 2 * i + 1, 4 )  = -pts_3[i].x( ) * pts_2[i].z( ) / pts_2[i].y( );
                M( 2 * i + 1, 5 )  = -pts_3[i].y( ) * pts_2[i].z( ) / pts_2[i].y( );
                M( 2 * i + 1, 6 )  = -pts_3[i].z( ) * pts_2[i].z( ) / pts_2[i].y( );
                M( 2 * i + 1, 7 )  = -pts_2[i].z( ) / pts_2[i].y( );
                M( 2 * i + 1, 8 )  = pts_3[i].x( );
                M( 2 * i + 1, 9 )  = pts_3[i].y( );
                M( 2 * i + 1, 10 ) = pts_3[i].z( );
                M( 2 * i + 1, 11 ) = 1;
                break;
            case 2:
                // Update matrix A using eq. 5
                M( 2 * i, 0 )      = pts_3[i].x( );
                M( 2 * i, 1 )      = pts_3[i].y( );
                M( 2 * i, 2 )      = pts_3[i].z( );
                M( 2 * i, 3 )      = 1;
                M( 2 * i, 4 )      = 0;
                M( 2 * i, 5 )      = 0;
                M( 2 * i, 6 )      = 0;
                M( 2 * i, 7 )      = 0;
                M( 2 * i, 8 )      = -pts_2[i].x( ) / pts_2[i].z( ) * pts_3[i].x( );
                M( 2 * i, 9 )      = -pts_2[i].x( ) / pts_2[i].z( ) * pts_3[i].y( );
                M( 2 * i, 10 )     = -pts_2[i].x( ) / pts_2[i].z( ) * pts_3[i].z( );
                M( 2 * i, 11 )     = -pts_2[i].x( ) / pts_2[i].z( );
                M( 2 * i + 1, 0 )  = 0;
                M( 2 * i + 1, 1 )  = 0;
                M( 2 * i + 1, 2 )  = 0;
                M( 2 * i + 1, 3 )  = 0;
                M( 2 * i + 1, 4 )  = pts_3[i].x( );
                M( 2 * i + 1, 5 )  = pts_3[i].y( );
                M( 2 * i + 1, 6 )  = pts_3[i].z( );
                M( 2 * i + 1, 7 )  = 1;
                M( 2 * i + 1, 8 )  = -pts_2[i].y( ) / pts_2[i].z( ) * pts_3[i].x( );
                M( 2 * i + 1, 9 )  = -pts_2[i].y( ) / pts_2[i].z( ) * pts_3[i].y( );
                M( 2 * i + 1, 10 ) = -pts_2[i].y( ) / pts_2[i].z( ) * pts_3[i].z( );
                M( 2 * i + 1, 11 ) = -pts_2[i].y( ) / pts_2[i].z( );
                break;
        }
    }
}

void
cv::DLT::solveDLT( )
{
    //        std::cout << " M " << std::endl << M << std::endl;

    Eigen::JacobiSVD< Eigen::MatrixXd > svd( M, Eigen::ComputeThinV );
    //    std::cout << " singularValues " << std::endl << svd.singularValues( ) <<
    //    std::endl;

    Eigen::MatrixXd h = svd.matrixV( ).block< 12, 1 >( 0, 11 ).transpose( );

    //        std::cout << " svd.matrixV( ) " << std::endl << svd.matrixV( ) << std::endl;

    for ( int i = 0; i < 3; i++ )
    {
        T( i ) = h( 0, 4 * i + 3 ); // Translation

        for ( int j = 0; j < 3; j++ )
            RR( i, j ) = h( 0, 4 * i + j ); // Rotation
    }

    //    std::cout << " h " << std::endl << h << std::endl;
    double norm = sqrt( h( 0, 8 ) * h( 0, 8 ) + h( 0, 9 ) * h( 0, 9 ) + h( 0, 10 ) * h( 0, 10 ) );

    if ( RR.determinant( ) < 0 ) // tz < 0
    {
        T /= -norm;
        RR = RR / -norm;
    }
    else
    {
        T /= norm;
        RR = RR / norm;
    }

    Eigen::JacobiSVD< Eigen::MatrixXd > svd2( RR, Eigen::ComputeThinU | Eigen::ComputeThinV );

    R = svd2.matrixU( ) * svd2.matrixV( ).transpose( );
}
