cmake_minimum_required(VERSION 2.8.12)
project(SINS)

# Set C++14
add_compile_options(-std=c++14)

# Third party dependencies
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Glog  REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})

# Make core library
file(GLOB_RECURSE ALL_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/Core/*/*.h
)
file(GLOB_RECURSE ALL_SRC_FILES
    ${PROJECT_SOURCE_DIR}/Core/*/*.cpp
)

#include_directories(${PROJECT_SOURCE_DIR}/core/*/)
add_library(${PROJECT_NAME}_Core SHARED ${ALL_SRC_FILES} ${ALL_HEADER_FILES})

# Make examples.
include_directories(${PROJECT_SOURCE_DIR}/Core/)
add_executable(StationaryCoarseAlignmentExample 
    Examples/StationaryCoarseAlignmentExample.cpp)
target_link_libraries(StationaryCoarseAlignmentExample
    ${PROJECT_NAME}_Core)

add_executable(CombineData 
    Tools/CombineData.cpp)
target_link_libraries(CombineData
    ${PROJECT_NAME}_Core)

add_executable(InsUpdateExample 
    Examples/InsUpdateExample.cpp)
target_link_libraries(InsUpdateExample
    ${PROJECT_NAME}_Core)
