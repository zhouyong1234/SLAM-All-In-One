cmake_minimum_required(VERSION 2.8.3)
project(gps_vio)

find_package(catkin_simple REQUIRED)


catkin_simple(ALL_DEPS_REQUIRED)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

cs_add_library(gps_vio_lib src/ISAMGraph.cpp src/ISAM2Graph.cpp src/SWGraph.cpp src/TDGraph.cpp)

cs_add_executable(GPSVIO_ros_node src/GPSVIO_ros_node.cpp)
target_link_libraries(GPSVIO_ros_node gps_vio_lib gtsam)



cs_install()
cs_export()

