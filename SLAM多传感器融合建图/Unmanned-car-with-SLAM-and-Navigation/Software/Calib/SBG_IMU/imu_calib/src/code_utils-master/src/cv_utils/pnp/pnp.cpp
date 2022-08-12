#include <code_utils/cv_utils/pnp/pnp.h>

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point )
: solved( false )
{
    //    std::cout << "image_point " << image_point.size( ) << std::endl;
    //    std::cout << "scene_point " << scene_point.size( ) << std::endl;

    bool is_planar = false;
    if ( scene_point.size( ) < 6 )
    {
        is_planar = true;
    }
    else
    {
        int size = scene_point.size( );
        Eigen::MatrixXd scene_points( size, 3 );
        for ( int index = 0; index < size; ++index )
        {
            scene_points( index, 0 ) = scene_point[index]( 0 );
            scene_points( index, 1 ) = scene_point[index]( 1 );
            scene_points( index, 2 ) = scene_point[index]( 2 );
        }
        Eigen::MatrixXd scene_points2 = scene_points * scene_points.transpose( );

        Eigen::VectorXd svd_si = Eigen::JacobiSVD< Eigen::MatrixXd >( scene_points2, //
                                                                      Eigen::EigenvaluesOnly )
                                 .singularValues( );

        //        Eigen::Vector3d svd_si = svd.singularValues( );
        //        std::cout << svd_si << std::endl;
        //        std::cout << "is_planar ? " << svd_si( 2 ) / svd_si( 1 ) << std::endl;

        if ( ( svd_si( 2 ) / svd_si( 1 ) ) < 0.001 )
            is_planar = true;
        else
            is_planar = false;
    }
    //    std::cout << "is_planar " << is_planar << std::endl;

    Eigen::Matrix3d R_cw;
    Eigen::Vector3d t_cw;

    if ( is_planar )
    {
        cv::Homography llpnp( image_point, scene_point );
        R_cw = llpnp.getR( );
        t_cw = llpnp.getT( );
    }
    else
    {
        cv::DLT llpnp( image_point, scene_point );
        R_cw = llpnp.getR( );
        t_cw = llpnp.getT( );
    }

    //    std::cout << "R_cw " << std::endl << R_cw << std::endl;
    //    std::cout << "t_cw " << std::endl << t_cw.transpose( ) << std::endl;

    npnp = new cv::NonlinearPnP( R_cw, t_cw, image_point, scene_point );
}

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
: solved( false )
{
    bool is_planar = false;
    if ( scene_point.size( ) < 6 )
    {
        is_planar = true;
    }
    else
    {
        int size = scene_point.size( );
        Eigen::MatrixXd scene_points( size, 3 );
        for ( int index = 0; index < size; ++index )
        {
            scene_points( index, 0 ) = scene_point[index]( 0 );
            scene_points( index, 1 ) = scene_point[index]( 1 );
            scene_points( index, 2 ) = scene_point[index]( 2 );
        }
        Eigen::MatrixXd scene_points2 = scene_points * scene_points.transpose( );

        Eigen::JacobiSVD< Eigen::MatrixXd > svd( scene_points2, Eigen::EigenvaluesOnly );

        //        std::cout << svd.singularValues( ) << std::endl;

        if ( svd.singularValues( )( 2 ) / svd.singularValues( )( 1 ) < 1e-3 )
            is_planar = true;
        else
            is_planar = false;
    }

    Eigen::Matrix3d R_cw;
    Eigen::Vector3d t_cw;
    if ( is_planar )
    {
        cv::Homography llpnp( image_point, scene_point );
        R_cw = llpnp.getR( );
        t_cw = llpnp.getT( );
    }
    else
    {
        cv::DLT llpnp( image_point, scene_point );
        R_cw = llpnp.getR( );
        t_cw = llpnp.getT( );
    }

    //    q_dst = Eigen::Quaterniond( R_cw );
    //    T_dst = t_cw;

    //    std::cout << "P_2 " << std::endl << t_cw.transpose( ) << std::endl;

    npnp = new cv::NonlinearPnP( R_cw, t_cw, image_point, scene_point );
    getRT( q_dst, T_dst );
}

cv::Pnp::Pnp( const std::vector< Eigen::Vector3d >& image_point,
              const std::vector< Eigen::Vector3d >& scene_point,
              const Eigen::Quaterniond& q_init,
              const Eigen::Vector3d& T_init,
              Eigen::Quaterniond& q_dst,
              Eigen::Vector3d& T_dst )
: solved( false )
{
    npnp = new cv::NonlinearPnP( q_init.toRotationMatrix( ), T_init, image_point, scene_point );
    getRT( q_dst, T_dst );
}

bool
cv::Pnp::getRT( Eigen::Quaterniond& q_dst, Eigen::Vector3d& T_dst )
{
    return npnp->getRT( T_dst, q_dst );
}
