cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_sparsifier)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  roscpp
  nodelet
  sensor_msgs)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare info that other packages need to import library generated here
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES laser_scan_sparsifier
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

#Create library
add_library(laser_scan_sparsifier src/laser_scan_sparsifier.cpp)
target_link_libraries( laser_scan_sparsifier ${catkin_LIBRARIES})
add_dependencies(laser_scan_sparsifier ${catkin_EXPORTED_TARGETS})

#Create nodelet
add_library(laser_scan_sparsifier_nodelet src/laser_scan_sparsifier_nodelet.cpp)
target_link_libraries(laser_scan_sparsifier_nodelet laser_scan_sparsifier)

#Create node
add_executable(laser_scan_sparsifier_node src/laser_scan_sparsifier_node.cpp)
target_link_libraries( laser_scan_sparsifier_node laser_scan_sparsifier )

#Install library
install(TARGETS laser_scan_sparsifier laser_scan_sparsifier_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#Install library includes
install(DIRECTORY include/laser_scan_sparsifier/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

#Install node
install(TARGETS laser_scan_sparsifier_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

#Install nodelet description
install(FILES laser_scan_sparsifier_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

#Install demo directory
install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
