cmake_minimum_required(VERSION 3.0.2)
project(hdl_global_localization)

option(ENABLE_TEASER "Build with Teaser++" OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED QUIET)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include(ExternalProject)
if(ENABLE_TEASER)
  ExternalProject_Add(teaser
    GIT_REPOSITORY https://github.com/koide3/TEASER-plusplus
    GIT_TAG master
    BUILD_IN_SOURCE
    CMAKE_ARGS -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_DOC=OFF -DBUILD_TEASER_FPFH=ON -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    INSTALL_COMMAND make install
  )

  set(TEASER_LIBRARIES
    ${CATKIN_DEVEL_PREFIX}/lib/libpmc.so
    ${CATKIN_DEVEL_PREFIX}/lib/libteaser_registration.so
    ${CATKIN_DEVEL_PREFIX}/lib/libteaser_features.so
  )

  add_definitions(-DTEASER_ENABLED)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  SetGlobalLocalizationEngine.srv
  SetGlobalMap.srv
  QueryGlobalLocalization.srv
)
generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hdl_global_localization
#  CATKIN_DEPENDS roscpp
#  DEPENDS teaser_registration
)

###########
## Build ##
###########

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(hdl_global_localization
  src/hdl_global_localization/bbs/bbs_localization.cpp
  src/hdl_global_localization/ransac/voxelset.cpp
  src/hdl_global_localization/ransac/ransac_pose_estimation.cpp
  src/hdl_global_localization/engines/global_localization_bbs.cpp
  src/hdl_global_localization/engines/global_localization_fpfh_ransac.cpp
)
target_link_libraries(hdl_global_localization
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

if(TEASER_LIBRARIES)
  add_library(hdl_global_localization_teaser
    src/hdl_global_localization/engines/global_localization_fpfh_teaser.cpp
  )
  add_dependencies(hdl_global_localization_teaser teaser)
  target_link_libraries(hdl_global_localization_teaser
    ${TEASER_LIBRARIES}
  )

  add_dependencies(hdl_global_localization teaser)
  add_dependencies(hdl_global_localization hdl_global_localization_teaser)
  target_link_libraries(hdl_global_localization
    hdl_global_localization_teaser
  )
endif()


add_executable(hdl_global_localization_node src/hdl_global_localization_node.cpp)
add_dependencies(hdl_global_localization_node hdl_global_localization ${PROJECT_NAME}_gencpp)
target_link_libraries(hdl_global_localization_node
  hdl_global_localization
  ${catkin_LIBRARIES}
)


add_executable(hdl_global_localization_test src/hdl_global_localization_test.cpp)
add_dependencies(hdl_global_localization_test ${PROJECT_NAME}_gencpp)
target_link_libraries(hdl_global_localization_test
  ${catkin_LIBRARIES}
)
