cmake_minimum_required(VERSION 3.10.2)
project(slam_demo)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
#     tf
#     pcl_ros
#     eigen_conversions
    message_generation
    std_srvs
)


generate_messages(
   DEPENDENCIES
   std_msgs
)

message(STATUS "[INFO]:opencv is searching....")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake_modules)
find_package( OpenCV REQUIRED)
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories(
    include ${catkin_INCLUDE_DIRS}
    "/usr/include/eigen3"
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../include
    ${PROJECT_SOURCE_DIR}/../include/DataPretreat
    ${PROJECT_SOURCE_DIR}/../include/FrontEnd
    ${PROJECT_SOURCE_DIR}/../include/BackEnd
    ${PROJECT_SOURCE_DIR}/../include/LoopClosing
    ${PROJECT_SOURCE_DIR}/../include/Viewer
    ${PROJECT_SOURCE_DIR}/../include/Tools
    ${CSPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
    ../src/DataPretreat/DepthMap.cc
    ../src/FrontEnd/FeatureMather.cc
    ../src/FrontEnd/EpipolarLine.cc
    ../src/FrontEnd/PoseSolver.cc
    ../src/BackEnd/Optimization.cc
)

target_link_libraries(${PROJECT_NAME}_lib
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CSPARSE_LIBRARY}
g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

add_executable(ros_node src/ros_node.cpp)
add_dependencies(ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

