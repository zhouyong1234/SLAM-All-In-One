#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
    VectorXd state(16);
    Vector4d q;
    Vector3d p;
    Vector3d v;
    Vector3d bg;
    Vector3d ba;
    q << 1.0, 0.0, 0.0, 0.0;
    p << 3.0, 2.0, 1.0;
    v << 1.23, 2.23, 3.21;
    bg << 0, 2.23, 3.21;
    ba << 0.12, 2.2, 3.21;
    state.segment(0,4)  = Vector4d(1.0, 0.0, 0.0, 0.0);
    state.segment(4,3)  = Vector3d(3.0, 2.0, 1.0);
    state.segment(7,3)  = Vector3d(1.23, 2.23, 3.21);
    state.segment(10,3) = Vector3d(0, 2.23, 3.21);
    state.segment(13,3) = Vector3d(1.2, 0.2, 0.2);

    cout << "--state = \n" << state << endl;

    MatrixXd s = MatrixXd::Zero(16,2);
    s.block(0,0,16,1) = state;
    cout << "s = \n" << s << endl;
    VectorXd afters(s);
    cout << "afters = " << afters << endl;

    int Xsize = state.rows();
    state.conservativeResize(Xsize + 10);

    //appending the current body pose
    state.segment<10>(Xsize) = state.head(10);

    cout << "--after.size = \n" << state.size() << endl;

    MatrixXd covariance = MatrixXd::Zero(15,15);
    cout << "--covariance size = " << covariance.size() << endl;

    MatrixXf matA(2, 2);
    matA << 1, 2, 3, 4;
    MatrixXf matB = matA;


    //matB << matA, matA/10, matA/10, matA;
    matB.block(matB.rows(),0,2,2) = matA;



    std::cout << matB << std::endl;

    Matrix<double,5,4> H;
    H << 1, 3.3, 2.4, 1.1,
        2.3,3.6, 1.37,2.35,
        3.67,2.35,2.56,1.65,
        0.23,2.34,9.1,5.1,
        5.2,2.3,5.12,5.5;
    cout << "H = \n" << H << endl;
    HouseholderQR<MatrixXd> qr(H);
    MatrixXd Q,R;
    Q = qr.householderQ() * (MatrixXd::Identity(H.rows(),H.cols()));
    R = qr.matrixQR().block(0,0,H.cols(),H.cols()).triangularView<Upper>();

    cout << "Q = \n" << Q << "\nR = \n" << R << endl;

    cout << "Q*R = \n" << Q * R << endl;







    return 0;
}
