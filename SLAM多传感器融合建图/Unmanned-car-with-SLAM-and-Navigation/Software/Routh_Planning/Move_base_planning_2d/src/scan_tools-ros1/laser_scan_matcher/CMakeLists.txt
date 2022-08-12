cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_matcher)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  roscpp
  nodelet
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} rostest)
find_package(PCL REQUIRED QUIET)

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

# Declare info that other packages need to import library generated here
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES laser_scan_matcher
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

#Create library
add_library(laser_scan_matcher src/laser_scan_matcher.cpp)

#Note we don't link against pcl as we're using header-only parts of the library
target_link_libraries( laser_scan_matcher ${catkin_LIBRARIES} ${csm_LIBRARIES})
add_dependencies(laser_scan_matcher ${csm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#Create nodelet
add_library(laser_scan_matcher_nodelet src/laser_scan_matcher_nodelet.cpp)
target_link_libraries(laser_scan_matcher_nodelet laser_scan_matcher)

#Create node
add_executable(laser_scan_matcher_node src/laser_scan_matcher_node.cpp)
target_link_libraries( laser_scan_matcher_node laser_scan_matcher )

#Install library
install(TARGETS laser_scan_matcher laser_scan_matcher_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#Install library includes
install(DIRECTORY include/laser_scan_matcher/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

#Install node
install(TARGETS laser_scan_matcher_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

#Install nodelet description
install(FILES laser_scan_matcher_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

#Install demo files
install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

add_rostest(test/run.test)
add_rostest(test/covariance.test)
