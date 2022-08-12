project(gtsam_test)
cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})
INCLUDE_DIRECTORIES("/usr/include/eigen3")

add_executable( gtsam_main gtsam_main.cpp )
target_link_libraries(gtsam_main ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable( isam_main isam_main.cpp )
target_link_libraries(isam_main ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable( gtsam_pose3 gtsam_pose3.cpp )
target_link_libraries(gtsam_pose3 ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable( isam_pose3 isam_pose3.cpp )
target_link_libraries(isam_pose3 ${Boost_LIBRARIES} -lgtsam -ltbb)