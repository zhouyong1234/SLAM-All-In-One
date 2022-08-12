cmake_minimum_required(VERSION 2.8.3)
project(map_conversion)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
)

set(ALL_TARGET_LIBRARIES "")

# find_package(gflags REQUIRED)

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/yaml.cmake)

include_directories( include
${catkin_INCLUDE_DIRS}
)

include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(local_environment_node src/app/local_environment_node.cpp ${ALL_SRCS})
add_dependencies(local_environment_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_environment_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} gflags yaml-cpp)

add_executable(global_submap_node src/app/global_submap_node.cpp ${ALL_SRCS})
add_dependencies(global_submap_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_submap_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} gflags yaml-cpp)

install(TARGETS 
    local_environment_node
    global_submap_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)

