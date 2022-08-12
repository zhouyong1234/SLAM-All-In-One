#ifndef NONLINEARPNP_H
#define NONLINEARPNP_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace cv
{

class Utility
{
    public:
    template< typename Derived >
    static Eigen::Matrix< typename Derived::Scalar, 3, 3 >
    skewSymmetric( const Eigen::MatrixBase< Derived >& q )
    {
        Eigen::Matrix< typename Derived::Scalar, 3, 3 > ans;
        ans << typename Derived::Scalar( 0 ), -q( 2 ), q( 1 ), q( 2 ),
        typename Derived::Scalar( 0 ), -q( 0 ), -q( 1 ), q( 0 ), typename Derived::Scalar( 0 );
        return ans;
    }
};

class NonlinearPnP
{
    class ReprojectionError
    {
        public:
        ReprojectionError( const Eigen::Vector3d& image_point, const Eigen::Vector3d& scene_point );

        template< typename T >
        bool operator( )( const T* const ex_paramt, T* residuals ) const
        {

            Eigen::Matrix< T, 3, 1 > _t_c;
            _t_c[0] = ex_paramt[0];
            _t_c[1] = ex_paramt[1];
            _t_c[2] = ex_paramt[2];

            Eigen::Quaternion< T > _q_cw( ex_paramt[6], ex_paramt[3], ex_paramt[4], ex_paramt[5] );
            _q_cw.normalize( );

            Eigen::Matrix< T, 3, 1 > p_w;
            p_w[0] = T( scene_point_.x( ) );
            p_w[1] = T( scene_point_.y( ) );
            p_w[2] = T( scene_point_.z( ) );

            Eigen::Matrix< T, 3, 1 > p_c;
            p_c = _q_cw * p_w + _t_c;
            p_c.normalize( );

            T r_x = p_c( 0 ) - T( image_point_( 0 ) );
            T r_y = p_c( 1 ) - T( image_point_( 1 ) );
            T r_z = p_c( 2 ) - T( image_point_( 2 ) );

            // clang-format off
             residuals[0] = T( tangent_base( 0, 0 ) ) * r_x
                          + T( tangent_base( 0, 1 ) ) * r_y
                          + T( tangent_base( 0, 2 ) ) * r_z;
             residuals[1] = T( tangent_base( 1, 0 ) ) * r_x
                          + T( tangent_base( 1, 1 ) ) * r_y
                          + T( tangent_base( 1, 2 ) ) * r_z;
            // clang-format on

            return true;
        }

        Eigen::Vector3d image_point_; // 2D to 3D point on image
        Eigen::Vector3d scene_point_; // 3D corresponding point
        Eigen::Matrix< double, 2, 3 > tangent_base;

        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    public:
    NonlinearPnP( const Eigen::Matrix3d& _R_initial,
                  const Eigen::Vector3d& _T_initial,
                  const std::vector< Eigen::Vector3d >& image_point,
                  const std::vector< Eigen::Vector3d >& scene_point );
    ~NonlinearPnP( ) {}

    bool getRT( Eigen::Vector3d& T_out, Eigen::Quaterniond& q_out );

    bool solved;
    Eigen::Quaterniond q;
    Eigen::Vector3d T;
    double data[9];
    int point_num;
};

class ProjectionFactor : public ceres::SizedCostFunction< 2, 7 >
{
    public:
    ProjectionFactor( const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j );
    virtual bool Evaluate( const double* const* ex_paramt, double* residuals, double** jacobians ) const;
    void check( double** parameters ) {}

    Eigen::Vector3d image_point_, scene_point_;
    Eigen::Matrix< double, 2, 3 > tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
}
#endif // NONLINEARPNP_H
