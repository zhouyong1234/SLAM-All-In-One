#ifndef DLT_H
#define DLT_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <iostream>

namespace cv
{

class DLT
{
    public:
    DLT( const std::vector< Eigen::Vector3d >& pts_2, const std::vector< Eigen::Vector3d >& pts_3 );

    public:
    Eigen::Matrix3d getR( ) const { return R; }
    Eigen::Vector3d getT( ) const { return T; }
    bool solved( ) const { return m_solve_ok; }

    private:
    void readPointsPlanar( const std::vector< Eigen::Vector3d >& pts_2,
                           const std::vector< Eigen::Vector3d >& pts_3 );
    void solveDLT( );

    private:
    Eigen::Matrix3d R;
    Eigen::Matrix3d RR;
    Eigen::Vector3d T;

    Eigen::MatrixXd M;
    Eigen::MatrixXd mat_tmp;
    int m_num_points;
    bool m_solve_ok;

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif // DLT_H
