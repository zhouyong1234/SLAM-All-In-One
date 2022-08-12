#ifndef TRIANGULATION_H
#define TRIANGULATION_H
#include "common_include.h"
#include "config.h"
#include "camera.h"
#include "types.h"
#include <ceres/ceres.h>


namespace MSCKF_MINE
{
class TriangulationReprojectionErr {
public:
    TriangulationReprojectionErr(const Vector2d &pixel)
        : pixel_(pixel) {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation,
                    const T* const point3d,
                    T* residuals) const
    {
        /*compute the residual*/
        /*it should be noted that the point3d is (x/z, y/z, 1/z)*/
        Eigen::Map<const Eigen::Quaternion<T> > q(quaternion);
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > t(translation);
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > pw(point3d);

        Eigen::Matrix<T, 3, 1> pc = q.matrix()*pw + t;


        T fx = Config::get<T>("Camera.fx");
        T fy = Config::get<T>("Camera.fy");
        T cx = Config::get<T>("Camera.cx");
        T cy = Config::get<T>("Camera.cy");

        T u = fx * pc(0) + cx;
        T v = fy * pc(1) + cy;

        residuals[0] = u - T(pixel_(0));
        residuals[1] = v - T(pixel_(1));

        return true;
    }

    static ceres::CostFunction* Create(
            const Vector2d &pixel) {
        return new ceres::AutoDiffCostFunction<TriangulationReprojectionErr, 2, 4, 3, 3>(
                    new TriangulationReprojectionErr(pixel));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    const Vector2d pixel_; /*observation*/

};

}


#endif // TRIANGULATION_H
