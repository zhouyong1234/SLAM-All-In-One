#include <code_utils/cv_utils/pnp/nonlinearpnp.h>

cv::NonlinearPnP::NonlinearPnP( const Eigen::Matrix3d& _R_initial,
                                const Eigen::Vector3d& _T_initial,
                                const std::vector< Eigen::Vector3d >& image_point,
                                const std::vector< Eigen::Vector3d >& scene_point )
: T( _T_initial )
{

    cv::ProjectionFactor::sqrt_info = Eigen::Matrix2d::Identity( );

    if ( image_point.size( ) != scene_point.size( ) )
        std::cerr << "Error of point size" << std::endl;
    q = _R_initial;

    // initialize the params to something close to the gt
    double ext[] = { T[0], T[1], T[2], q.x( ), q.y( ), q.z( ), q.w( ) };

    point_num = image_point.size( );

    ceres::Problem problem;

    for ( int i = 0; i < point_num; ++i )
    {

        // ProjectionFactor* f = new ProjectionFactor( image_point[i], scene_point[i] );

        ceres::CostFunction* f = new ceres::AutoDiffCostFunction< ReprojectionError, 2, 7 >(
        new ReprojectionError( image_point[i], scene_point[i] ) );

        problem.AddResidualBlock( f, NULL /* squared loss */, ext );
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.logging_type                 = ceres::SILENT;
    options.trust_region_strategy_type   = ceres::DOGLEG;
    //    options.max_num_iterations         = 5;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    //    std::cout << summary.FullReport( ) << "\n";

    q.w( ) = ext[6];
    q.x( ) = ext[3];
    q.y( ) = ext[4];
    q.z( ) = ext[5];
    T << ext[0], ext[1], ext[2];

    if ( summary.final_cost > 1 )
        solved = false;
    else
        solved = true;
}

bool
cv::NonlinearPnP::getRT( Eigen::Vector3d& T_out, Eigen::Quaterniond& q_out )
{
    q_out = q.normalized( );
    T_out = T;

    if ( solved )
        return true;
    else
        return false;
}

cv::NonlinearPnP::ReprojectionError::ReprojectionError( const Eigen::Vector3d& image_point,
                                                        const Eigen::Vector3d& scene_point )
: image_point_( image_point )
, scene_point_( scene_point )
{
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d tmp( 0, 0, 1 );
    Eigen::Vector3d a = scene_point_.normalized( );
    if ( a == tmp )
        tmp << 1, 0, 0;

    b1 = ( tmp - a * ( a.transpose( ) * tmp ) ).normalized( );
    b2 = a.cross( b1 );

    tangent_base.block< 1, 3 >( 0, 0 ) = b1.transpose( );
    tangent_base.block< 1, 3 >( 1, 0 ) = b2.transpose( );
}

cv::ProjectionFactor::ProjectionFactor( const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j )
: image_point_( _pts_i )
, scene_point_( _pts_j )
{
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = scene_point_.normalized( );
    Eigen::Vector3d tmp( 0, 0, 1 );
    if ( a == tmp )
        tmp << 1, 0, 0;
    b1 = ( tmp - a * ( a.transpose( ) * tmp ) ).normalized( );
    b2 = a.cross( b1 );
    tangent_base.block< 1, 3 >( 0, 0 ) = b1.transpose( );
    tangent_base.block< 1, 3 >( 1, 0 ) = b2.transpose( );
}

Eigen::Matrix2d cv::ProjectionFactor::sqrt_info;

bool
cv::ProjectionFactor::Evaluate( const double* const* ex_paramt, double* residuals, double** jacobians ) const
{

    Eigen::Vector3d _t_c( ex_paramt[0][0], ex_paramt[0][1], ex_paramt[0][2] );
    Eigen::Quaterniond _q_cw( ex_paramt[0][6], ex_paramt[0][3], ex_paramt[0][4], ex_paramt[0][5] );
    //    _q_cw.normalize( );

    Eigen::Matrix< double, 3, 1 > p_c;
    p_c = _q_cw * scene_point_ + _t_c;

    Eigen::Map< Eigen::Vector2d > residual( residuals );

    residual = tangent_base * ( p_c.normalized( ) - image_point_ );
    //    residual = sqrt_info * residual;

    if ( jacobians )
    {
        Eigen::Matrix3d R_cw = _q_cw.toRotationMatrix( );
        Eigen::Matrix< double, 2, 3 > reduce( 2, 3 );

        double norm = p_c.norm( );
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = p_c( 0 );
        x2 = p_c( 1 );
        x3 = p_c( 2 );

        // clang-format off
        norm_jaco <<
            1.0 / norm - x1 * x1 / pow( norm, 3 ), -x1 * x2 / pow( norm, 3 ), -x1 * x3 / pow( norm, 3 ),
            -x1 * x2 / pow( norm, 3 ), 1.0 / norm - x2 * x2 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ),
            -x1 * x3 / pow( norm, 3 ), -x2 * x3 / pow( norm, 3 ), 1.0 / norm - x3 * x3 / pow( norm, 3 );
        // clang-format on

        if ( jacobians[0] )
        {
            Eigen::Map< Eigen::Matrix< double, 2, 7, Eigen::RowMajor > > jacobian_pose_i( jacobians[0] );
            // clang-format on
            Eigen::Matrix< double, 3, 6 > jaco_i;
            jaco_i.leftCols< 3 >( )  = Eigen::Matrix3d::Identity( );
            jaco_i.rightCols< 3 >( ) = -R_cw * Utility::skewSymmetric( scene_point_ );
            // clang-format on

            jacobian_pose_i.leftCols< 6 >( ) = tangent_base * jaco_i;
            jacobian_pose_i.rightCols< 1 >( ).setZero( );
        }
    }
    return true;
}
