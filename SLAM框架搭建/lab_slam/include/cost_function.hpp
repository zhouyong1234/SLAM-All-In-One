#ifndef COST_FUNCTION_HPP
#define COST_FUNCTION_HPP
/*
 * Function:
 * 给定corner/plane特征点，以及参数（主要是最近邻或者几何参数），针对同一个优化变量（点云的相对位姿），建立约束方程（单边的约束）。
 * 在方法上：
 * （1）对于corner点，可以找到两个点成线，计算点和线之间的距离，
 * 这两个点可以像LOAM来自最近邻和异线的次近邻，也可以通过直线拟合，由均值和协方差的特征向量虚构一对；
 * （2）对于面特征，可以像LOAM找到最近的3个异线点组成面，面上的两个向量，计算出面的法向，误差为当前点与一点之间的连线在法向上的投影，
 * 另一种方式是拟合平面方程，通过点到面的高度公式进行计算。
 * 同时，误差函数还可以指定插值的参数s.
 * 另外，建立约束之前需要验证线、面约束的合理性，相对应的方法包括最近邻是否足够近，直线协方差分解后特征值的比例，面拟合后点到高的初始大小足够小。
 * */

#include <ceres/ceres.h>
#include <Eigen/Dense>

// 如果不需要插值进行畸变校正，那么s不赋值即可
struct LineFactor{
    LineFactor(Eigen::Vector3d pa, Eigen::Vector3d pb, Eigen::Vector3d p, double s):last_point_a(pa), \
        last_point_b(pb), current_point(p), inter_coeff(s){
        ab_norm = (last_point_b - last_point_a).norm();
    }

    template <typename T>
    bool operator()(const T* q, const T* t, T* residual) const {
        Eigen::Quaternion<T> current_q(q[3], q[0], q[1], q[2]);// 要注意顺序
        Eigen::Quaternion<T> identity_q = Eigen::Quaternion<T>(T(1), T(0), T(0), T(0));
        current_q = identity_q.slerp(T(inter_coeff), current_q);// 处处注意T
        Eigen::Matrix<T, 3, 1> current_t(T(inter_coeff) * t[0], T(inter_coeff) * t[1], T(inter_coeff) * t[2]);

        Eigen::Matrix<T, 3, 1> cp(T(current_point.x()), T(current_point.y()), T(current_point.z()));
        Eigen::Matrix<T, 3, 1> current_global_point = current_q * cp + current_t;

        Eigen::Matrix<T, 3, 1> lpa(T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z()));
        Eigen::Matrix<T, 3, 1> lpb(T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z()));

        Eigen::Matrix<T, 3, 1> height = (lpb - lpa).cross(current_global_point - lpa);
        residual[0] = height(0) / T(ab_norm);
        residual[1] = height(1) / T(ab_norm);
        residual[2] = height(2) / T(ab_norm);
//        Eigen::Matrix<T, 3, 1> nu = (current_global_point - lpa).cross(current_global_point - lpb);// 为什么不直接取模长,下面直接取高呢?
//        Eigen::Matrix<T, 3, 1> de = lpa - lpb;
//
//        residual[0] = nu.x() / de.norm();
//        residual[1] = nu.y() / de.norm();
//        residual[2] = nu.z() / de.norm();
//        LOG(INFO) << "Error: " << residual[0] << ", " << residual[1] << ", " << residual[2];
        return true;
    }

    static ceres::CostFunction* create(Eigen::Vector3d pa, Eigen::Vector3d pb, Eigen::Vector3d p, double s = 1.0){
        return new ceres::AutoDiffCostFunction<LineFactor, 3, 4, 3>(new LineFactor(pa, pb, p, s));
    }
    Eigen::Vector3d last_point_a, last_point_b, current_point;
    double inter_coeff;
    double ab_norm;
};

// For every given point, and plane {plane_norm, d} with plane_norm normalized in advance,
// the error is plane_norm.dot(point) + d
struct PlaneFactor{
    PlaneFactor(Eigen::Vector3d plane_norm, double dis, Eigen::Vector3d input_point, double s):plane_norm_(plane_norm),
        dis_(dis), current_point_(input_point), inter_coeff_(s){}

    template <typename T>
    bool operator()(const T* q, const T* t, T* residual) const {
        Eigen::Quaternion<T> current_q(q[3], q[0], q[1], q[2]);
        Eigen::Quaternion<T> identiy_q(T(1), T(0), T(0), T(0));
        current_q = identiy_q.slerp(T(inter_coeff_), current_q);// 处处注意T
        Eigen::Matrix<T, 3, 1> current_t(T(inter_coeff_) * t[0], T(inter_coeff_) * t[1], T(inter_coeff_) * t[2]);
        Eigen::Matrix<T, 3, 1> current_global_point(T(current_point_.x()), T(current_point_.y()), T(current_point_.z()));
        current_global_point = current_q * current_global_point + current_t;

        Eigen::Matrix<T, 3, 1> plane_norm(T(plane_norm_.x()), T(plane_norm_.y()), T(plane_norm_.z()));
        residual[0] = plane_norm.dot(current_global_point) + T(dis_);
        return true;
    }

    // 要求plane_norm是平面的单位法向量
    //TODO: 一定要梳理清楚所有的输入和输出，不要遗漏
    static ceres::CostFunction* create(Eigen::Vector3d plane_norm, double dis, Eigen::Vector3d input_point, double s = 1){
        return new ceres::AutoDiffCostFunction<PlaneFactor, 1, 4, 3>(new PlaneFactor(plane_norm, dis, input_point, s));
    }

    Eigen::Vector3d plane_norm_;
    double dis_;
    Eigen::Vector3d current_point_;
    double inter_coeff_;
};

#endif