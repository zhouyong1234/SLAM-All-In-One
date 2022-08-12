cmake_minimum_required(VERSION 2.8.3)
project(one_liom)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(rosbagregist src/rosbagregist.cpp)
target_link_libraries(rosbagregist ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(map src/map.cpp)
target_link_libraries(map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})


