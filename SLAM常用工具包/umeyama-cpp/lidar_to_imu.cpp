#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>



int main() {

    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();


    Eigen::Matrix4d vehicle_to_lidar_right;
    vehicle_to_lidar_right <<     -0.512695,  0.700506, -0.496422, -0.436669,
                                  -0.497416, -0.713622, -0.493276, -0.411346,
                                  -0.6998, -0.00597189,  0.714313,  1.94785,
                                   0.000000,  0.000000,  0.000000,  1.00000;
    
                                
    Eigen::Matrix4d vehicle_to_lidar_left;
    vehicle_to_lidar_left <<       -0.515105, -0.702383, -0.491249, -0.438343,
                                    0.487008, -0.711468, 0.506593,   0.395882,
                                   -0.70533, 0.0217062, 0.708547,    1.94095,
                                    0.000000,  0.000000,  0.000000,  1.00000;


      
    Eigen::Matrix4d vehicle_to_imu;
    vehicle_to_imu    <<          1.0, 0.0, 0.0, -0.07,
                                  0.0, 1.0, 0.0,  0.0,
                                  0.0, 0.0, 1.0,  1.7,
                                  0.0, 0.0, 0.0,  1.0;


    Eigen::Matrix4d right_lidar_to_imu;
    Eigen::Matrix4d left_lidar_to_imu;

    right_lidar_to_imu = vehicle_to_imu.inverse() * vehicle_to_lidar_right;
    left_lidar_to_imu = vehicle_to_imu.inverse() *vehicle_to_lidar_left;

    // imu_to_lidar = vehicle_to_lidar.inverse() * vehicle_to_imu;

    std::cout << "right_lidar_to_imu: " << std::endl;

    std::cout << right_lidar_to_imu << std::endl;

    std::cout << "left_lidar_to_imu: " << std::endl;

    std::cout << left_lidar_to_imu << std::endl;


    // Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
    // rotation_matrix1 << -0.512695, 0.700506, -0.496422,  
    //                     -0.497416, -0.713622, -0.493276, 
    //                     -0.6998, -0.00597189, 0.714313;


    // Eigen::Vector3d t1;
    // t1 <<  -0.436669, -0.411346, 1.94785;

    // T1=Eigen::Isometry3d::Identity();
    // T1.rotate ( rotation_matrix1 );
    // T1.pretranslate ( t1 );


    // Eigen::Matrix3d rotation_matrix2 = Eigen::Matrix3d::Identity();


    // Eigen::Vector3d t2;
    // t2 <<  -0.07, 0, 1.7;
    // T2=Eigen::Isometry3d::Identity();
    // T2.rotate ( rotation_matrix2 );
    // T2.pretranslate ( t2 );

    // // T = T2.inverse() * T1;

    // T = T1.inverse() * T2;

    // std::cout << "T: " << std::endl;

    // std::cout << T.matrix() << std::endl;

  return 0;
}