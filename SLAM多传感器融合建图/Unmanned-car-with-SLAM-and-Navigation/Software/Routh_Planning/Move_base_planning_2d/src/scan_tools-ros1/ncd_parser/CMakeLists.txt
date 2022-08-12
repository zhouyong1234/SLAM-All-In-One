cmake_minimum_required(VERSION 2.8.3)
project(ncd_parser)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
    roscpp
    sensor_msgs
    tf )

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare info that other packages need to import library generated here
catkin_package()

#Create node
add_executable( ${PROJECT_NAME} src/ncd_parser.cpp)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#Install node
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

#Install demo directory
install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
