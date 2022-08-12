#include "Alignment/StationaryCoarseAlignment.h"
#include "Utils/SO3.h"

#include <iostream>

using namespace SINS;

int main(int argc, char **argv) {
    // Start.
    // const Eigen::Vector3d gyro(4.85072550613717e-07, -2.57168325026221e-07, 4.78749072084102e-07);
    // const Eigen::Vector3d acc(-0.00220864776504132, -0.000983490241384058, 0.0980537484095091);

    // End.
    const Eigen::Vector3d gyro(-5.17745291091245e-07,	2.09718176638119e-07,	4.59047968234761e-07);
    const Eigen::Vector3d acc(-0.00230025125136539,	-7.95820184872582e-06,	0.0980495373619216);

    Eigen::Matrix3d C_nb = SINS::StationaryCoarseAlign(acc, gyro); // 
    const Eigen::Vector3d euler_nb = MatToAtt(C_nb); 

    constexpr double kRadToDeg = 180. / M_PI;    
    std::cout << "C_nb:\n" << std::fixed << C_nb << std::endl << std::endl << std::endl;
    std::cout << "euler_nb: " << std::fixed << euler_nb.transpose() * kRadToDeg << std::endl << std::endl;

    double latitude = SINS::ComputeLatitude(acc, gyro);
    std::cout << "Latitude: " << std::fixed << latitude * kRadToDeg << std::endl;

    return EXIT_SUCCESS;
}