cmake_minimum_required(VERSION 2.8.3)
project(polar_scan_matcher)

set( ROS_CXX_DEPENDENCIES
  roscpp
  tf
  sensor_msgs
  geometry_msgs)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES polar_scan_matcher
    CATKIN_DEPENDS ${ROSS_CXX_DEPENDENCIES}
)

add_library(polar_scan_matcher src/polar_match.cpp)

add_executable(psm_node src/psm_node.cpp)
target_link_libraries(psm_node polar_scan_matcher
                               ${catkin_LIBRARIES})

install(TARGETS polar_scan_matcher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#Install library includes
install(DIRECTORY include/polar_scan_matcher/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

install(TARGETS psm_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_executable(polar_scan_matcher src/psm_node.cpp 
#                                           src/polar_match.cpp)

