cmake_minimum_required(VERSION 2.8.3)
project(scan_to_cloud_converter)


# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  roscpp
  pcl_ros
  pcl_conversions)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(PCL REQUIRED QUIET)

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Declare info that other packages need to import library generated here
catkin_package( )

#Create node
add_executable( scan_to_cloud_converter_node
    src/scan_to_cloud_converter_node.cpp
    src/scan_to_cloud_converter.cpp )

# No need to link against pcl (using header only libraries)
target_link_libraries( scan_to_cloud_converter_node ${catkin_LIBRARIES})

add_dependencies(scan_to_cloud_converter_node ${catkin_EXPORTED_TARGETS})


#Install node
install(TARGETS scan_to_cloud_converter_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
