#include <Eigen/Dense>
namespace kinetic {
  /** \brief euler angle convert to Rotation Matrix Rwb
   *  @param eulerRad euler angle at rad 
   *  @param Rwb output rotation matrix which is rotation from body to world 
   *  @note Rwb = (Rbw)' Rbw = R(x)R(y)R(z) 
   */
  static void euler2RotationMatrix(const Eigen::Vector3d& eulerRad,Eigen::Matrix3d& Rwb) {
    double roll = eulerRad[0];
    double pitch = eulerRad[1];
    double yaw = eulerRad[2];
    double cr = cos(roll),sr = sin(roll);
    double cp = cos(pitch),sp = sin(pitch);
    double cy = cos(yaw),sy = sin(yaw);
    //w->b: z -> y -> x
    Eigen::Matrix3d Rz,Ry,Rx;
    Rz <<  cy, sy, 0.,
          -sy, cy, 0.,
           0., 0.,1.0;
    Ry <<  cp, 0.,-sp,
           0., 1., 0.,
           sp, 0., cp;
    Rx <<  1., 0., 0.,
           0., cr, sr,
           0.,-sr, cr;
    Eigen::Matrix3d Rbw = Rx * Ry * Rz;
    Rwb = Rbw.inverse();
  }
  /** @brief transform euler angle rate into body rate
    * @param eulerRad - euler rate in rad 
    * @return R - transform matrix 
   */ 
  static void eulerRate2BodyRate(const Eigen::Vector3d& eulerRad,Eigen::Matrix3d& R) {
    double roll = eulerRad[0];
    double pitch = eulerRad[1];
    double cr = cos(roll),sr = sin(roll);
    double cp = cos(pitch),sp = sin(pitch);
    R << 1.0, 0.,  -sp,
          0., cr, cp*sr,
          0.,-sr, cp*cr;
  }
}