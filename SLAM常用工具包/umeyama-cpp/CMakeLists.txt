cmake_minimum_required(VERSION 3.5)
project(umeyama)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
add_executable(umeyama
        main.cc
        umeyama.h umeyama.cc
)


add_executable(lidar_to_imu lidar_to_imu.cpp)
target_link_libraries(lidar_to_imu ${OpenCV_LIBS})

target_link_libraries(umeyama ${OpenCV_LIBS})
