cmake_minimum_required(VERSION 2.8.3)
project(msckf_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-w)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    )

find_package(OpenCV REQUIRED)

find_package(Eigen3)

add_subdirectory(third_party/GeographicLib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  "/home/dfz/dfz/master/Code/msckf/devel/include"
  "/usr/include/eigen3"
  third_party/GeographicLib/include
)

catkin_package()

add_executable(msckf_estimator
    src/estimator_node.cpp
	src/parameters.cpp
	src/estimator.cpp
)
target_link_libraries(msckf_estimator ${catkin_LIBRARIES} ${OpenCV_LIBS} libGeographiccc) 