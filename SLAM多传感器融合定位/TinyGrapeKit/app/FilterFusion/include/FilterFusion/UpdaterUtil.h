#pragma once

#include <Eigen/Core>

#include <FilterFusion/State.h>

namespace FilterFusion {

void LeftNullspaceProjection(const Eigen::MatrixXd& Hx, 
                             const Eigen::MatrixXd& Hf, 
                             const Eigen::VectorXd& res, 
                             Eigen::MatrixXd* H,
                             Eigen::VectorXd* r);

void CompressMeasurement(const Eigen::MatrixXd& H, 
                         const Eigen::VectorXd& r, 
                         Eigen::MatrixXd* H_cmp, 
                         Eigen::VectorXd* r_cmp);

void EKFUpdate(const Eigen::MatrixXd& H, 
               const Eigen::VectorXd& r, 
               const Eigen::MatrixXd& V,
               State* state);

} // namespace FilterFusion