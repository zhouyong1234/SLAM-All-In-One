cmake_minimum_required(VERSION 2.8.3)
project(gps_vio)

find_package(catkin REQUIRED COMPONENTS
roscpp
message_filters
nav_msgs
)

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
find_package(cmake_modules REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps_vio
  CATKIN_DEPENDS nav_msgs message_filters
#  DEPENDS eigen
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(gps_vio_lib
src/ISAMGraph.cpp
src/ISAM2Graph.cpp
src/SWGraph.cpp
src/TDGraph.cpp
)

add_executable(GPSVIO_ros_node src/GPSVIO_ros_node.cpp)

target_link_libraries(GPSVIO_ros_node gps_vio_lib gtsam ${catkin_LIBRARIES})

