#include <FilterFusion/UpdaterUtil.h>

#include <Eigen/Dense>

#include <TGK/Util/Util.h>

#include <glog/logging.h>
#include <iomanip>

namespace FilterFusion {


// QR分解 
void LeftNullspaceProjection(const Eigen::MatrixXd& Hx, 
                             const Eigen::MatrixXd& Hf, 
                             const Eigen::VectorXd& res, 
                             Eigen::MatrixXd* H,
                             Eigen::VectorXd* r) {
    const size_t rows = Hf.rows();
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(Hf);
    const Eigen::MatrixXd Q = qr.householderQ();
    const Eigen::MatrixXd Q2_trans = Q.rightCols(rows - 3).transpose();

    *H = Q2_trans * Hx;
    *r = Q2_trans * res;
}

// QR分解
void CompressMeasurement(const Eigen::MatrixXd& H, 
                         const Eigen::VectorXd& r, 
                         Eigen::MatrixXd* H_cmp, 
                         Eigen::VectorXd* r_cmp) {
    if (H.rows() <= H.cols()) {
        *H_cmp = H;
        *r_cmp = r;
        return;
    }
    
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(H);
    Eigen::MatrixXd Q = qr.householderQ();
    Eigen::MatrixXd Q1_trans = Q.leftCols(H.cols()).transpose();

    *H_cmp = qr.matrixQR().topRows(H.cols()).triangularView<Eigen::Upper>();
    *r_cmp = Q1_trans * r;
}

void LimitMinDiagValue(const double min_diag_val, Eigen::MatrixXd* mat) {
    for (size_t i = 0; i < mat->rows(); ++i) {
        if ((*mat)(i, i) < min_diag_val) {
            (*mat)(i, i) = min_diag_val;
        }
    }
}

void EKFUpdate(const Eigen::MatrixXd& H, 
               const Eigen::VectorXd& r, 
               const Eigen::MatrixXd& V,
               State* state) {
    const Eigen::MatrixXd P_minus = state->covariance;
    const Eigen::MatrixXd H_trans = H.transpose();
    const Eigen::MatrixXd S = H * P_minus * H_trans + V;
    const Eigen::MatrixXd S_inv = S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));   
    const Eigen::MatrixXd K = P_minus * H_trans * S_inv;

    // Compute delta x.
    const Eigen::VectorXd delta_x = K * r;

    // Update.
    state->Update(delta_x);
    
    // Update covariance.
    const Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(state->covariance.rows(), state->covariance.rows()) - K * H;
    state->covariance = I_KH * P_minus * I_KH.transpose() + K * V * K.transpose();

    state->covariance = state->covariance.eval().selfadjointView<Eigen::Upper>();
    LimitMinDiagValue(1e-12, &state->covariance);
}

}  // namespace FilterFusion