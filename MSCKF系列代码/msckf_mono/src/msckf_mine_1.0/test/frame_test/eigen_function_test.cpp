#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    AngleAxisd rotation_vector(M_PI/4, Vector3d(0, 0, 1));
    Matrix3d rotation_matrix = rotation_vector.matrix();
    cout <<"Rotation matrix = \n" << rotation_matrix << endl;

    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_matrix);

    T.pretranslate(Vector3d(1, 3, 4));
    cout << "Transform matrix = \n" << T.matrix() << endl;

    cout << "     T.linear = \n" << T.linear() << endl;
    cout << "T.translation = \n" << T.translation() << endl;


    return 0;
}
