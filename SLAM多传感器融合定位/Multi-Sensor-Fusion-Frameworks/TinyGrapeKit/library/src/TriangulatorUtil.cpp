#include <Geometry/TriangulatorUtil.h>

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <Camera/Camera.h>

namespace TGK {
namespace Geometry {

bool TriangulateDLT(const std::vector<Eigen::Matrix3d>& C_R_Gs, 
                    const std::vector<Eigen::Vector3d>& C_p_Gs,
                    const std::vector<Eigen::Vector2d>& NSP_points,
                    Eigen::Vector3d* G_p) {
    const size_t n_obs = C_R_Gs.size();
    Eigen::MatrixXd A(2 * n_obs, 4);
    Eigen::Matrix<double, 1, 4> P1, P2, P3;
    size_t idx = 0;
    for (size_t i = 0; i < n_obs; ++i) {
        const auto& pt = NSP_points[i];
        const double x = pt[0];
        const double y = pt[1];

        const auto&  R = C_R_Gs[i];
        const auto&  p = C_p_Gs[i];
        P1 << R.block<1, 3>(0, 0), p(0);
        P2 << R.block<1, 3>(1, 0), p(1);
        P3 << R.block<1, 3>(2, 0), p(2);
        
        A.block<1, 4>(idx, 0) = x * P3 - P1;
        ++idx;
        A.block<1, 4>(idx, 0) = y * P3 - P2;
        ++idx;
    }

    // Solve Ax = 0.
    const Eigen::Matrix4d H = A.transpose() * A;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(H);
    if (eigen_solver.info() != Eigen::Success) {
        LOG(WARNING) << "[TriangulateDLT]: Failed to compute eigenvector of H.";
        return false;
    }
    const Eigen::Vector4d X = eigen_solver.eigenvectors().leftCols<1>();

    if (std::abs(X[3]) < 1e-12) {
        LOG(WARNING) << "[TriangulateDLT]: X[3] is too small!";
        return false;
    }
    *G_p = X.topRows<3>() / X[3];

    return true;                     
}

class ProjectionCostFunc : public ceres::SizedCostFunction<2, 3> {
public:
    ProjectionCostFunc(const Camera::CameraPtr camera,
                       const Eigen::Matrix3d& C_R_G, 
                       const Eigen::Vector3d& C_p_G, 
                       const Eigen::Vector2d& im_pt)
        : camera_(camera), C_R_G_(C_R_G), C_p_G_(C_p_G), im_pt_(im_pt) { }

  virtual ~ProjectionCostFunc() { }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    const bool compute_jacobian = (jacobians != nullptr && jacobians[0] != nullptr);
    
    const Eigen::Map<const Eigen::Vector3d> G_p(parameters[0]);
    const Eigen::Vector3d C_p = C_R_G_ * G_p + C_p_G_;
    Eigen::Vector2d exp_im_pt;
    Eigen::Matrix<double, 2, 3> J_Ip_wrt_Cp;
    if (!camera_->CameraToImage(C_p, &exp_im_pt, compute_jacobian ? &J_Ip_wrt_Cp : nullptr)) {
        return false;
    }

    // Compute residual.
    Eigen::Map<Eigen::Vector2d> res_vec(residuals);
    res_vec = im_pt_ - exp_im_pt;

    // Compute the Jacobian if asked for.
    if (compute_jacobian) {
        // The jacobian pointer is defined in row-major.
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_wrt_G_p(jacobians[0]);

        const Eigen::Matrix3d& J_Cp_wrt_Gp = C_R_G_;
        // Add negative for the optimization problem.
        J_wrt_G_p = -J_Ip_wrt_Cp * J_Cp_wrt_Gp;
    }

    return true;
  }

private:
    const Camera::CameraPtr camera_;
    const Eigen::Matrix3d C_R_G_;
    const Eigen::Vector3d C_p_G_;
    const Eigen::Vector2d im_pt_;
};

bool RefineGlobalPoint(const Camera::CameraPtr camera,
                       const std::vector<Eigen::Matrix3d>& C_R_Gs, 
                       const std::vector<Eigen::Vector3d>& C_p_Gs,
                       const std::vector<Eigen::Vector2d>& im_points,
                       Eigen::Vector3d* G_p) {
    // Build the problem.
    ceres::Problem problem;
    
    // Add cost functions.
    for (size_t i = 0; i < C_R_Gs.size(); ++i) {
        ceres::CostFunction* cost_func = new ProjectionCostFunc(camera, C_R_Gs[i], C_p_Gs[i], im_points[i]);
        problem.AddResidualBlock(cost_func, new ceres::HuberLoss(std::sqrt(5.99)), G_p->data());
    }

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary); 

    if (summary.termination_type == ceres::FAILURE) { return false; } 

    return true;        
}

}  // namespace Geometry
}  // namespace TGK