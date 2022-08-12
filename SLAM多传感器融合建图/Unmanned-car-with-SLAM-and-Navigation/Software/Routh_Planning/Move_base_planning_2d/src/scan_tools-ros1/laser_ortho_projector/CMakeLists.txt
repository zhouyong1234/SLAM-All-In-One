cmake_minimum_required(VERSION 2.8.3)
project(laser_ortho_projector)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  roscpp
  nodelet
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
  geometry_msgs
  message_filters)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(PCL REQUIRED QUIET)

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Declare info that other packages need to import library generated here
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES laser_ortho_projector
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES} )

#Create library
add_library(laser_ortho_projector src/laser_ortho_projector.cpp)

#Note we don't link against pcl as we're using header-only parts of the library
target_link_libraries( laser_ortho_projector ${catkin_LIBRARIES})
add_dependencies(laser_ortho_projector ${catkin_EXPORTED_TARGETS})

#Create nodelet
add_library(laser_ortho_projector_nodelet src/laser_ortho_projector_nodelet.cpp)
target_link_libraries(laser_ortho_projector_nodelet laser_ortho_projector)

#Create node
add_executable(laser_ortho_projector_node src/laser_ortho_projector_node.cpp)
target_link_libraries( laser_ortho_projector_node laser_ortho_projector )

#Install library
install(TARGETS laser_ortho_projector laser_ortho_projector_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#Install library includes
install(DIRECTORY include/laser_ortho_projector/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

#Install node
install(TARGETS laser_ortho_projector_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

#Install nodelet description
install(FILES laser_ortho_projector_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

#Install demo files
install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
