cmake_minimum_required(VERSION 2.8)
project(eigen_lesson)
set(CMAKE_INSTALL_PREFIX "../../env/")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_executable(quaternion_slerp app/quaternion_slerp.cpp)
target_link_libraries(quaternion_slerp ${OpenCV_LIBS})

add_executable(quaternion_angle_distance app/quaternion_angle_distance.cpp)

