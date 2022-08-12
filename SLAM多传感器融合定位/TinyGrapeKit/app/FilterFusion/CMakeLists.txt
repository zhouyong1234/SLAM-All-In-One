cmake_minimum_required(VERSION 2.8.8)
project(FilterFusion)

# Set C++14
add_compile_options(-std=c++14)


# Set cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Third party dependencies
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Glog  REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}/opencv4)

# TinyGrapeKit library
set(TGK_DIR ${CMAKE_SOURCE_DIR}/../../library/install)
include_directories(${TGK_DIR}/include)
include_directories(${TGK_DIR}/include/TGK)
link_directories(${TGK_DIR}/lib)

# Pangolin
set(Pangolin_DIR ${CMAKE_SOURCE_DIR}/../../third_party/Pangolin)
include_directories(${Pangolin_DIR}/include)
include_directories(${Pangolin_DIR}/build/src/include)
link_directories(${Pangolin_DIR}/build/src)

# GeographicLib
set(GeoGraphic_DIR ${CMAKE_SOURCE_DIR}/../../third_party/GeographicLib)
include_directories(${GeoGraphic_DIR}/include)
link_directories(${GeoGraphic_DIR}/build)

# Make library
file(GLOB_RECURSE ALL_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/*.h
)
file(GLOB_RECURSE ALL_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED ${ALL_SRC_FILES} ${ALL_HEADER_FILES})
target_link_libraries(${PROJECT_NAME} TinyGrapeKit pangolin libGeographiccc)

## Declare C++ executables
add_executable(RunKAISTData example/RunKAISTData.cpp)
target_link_libraries(RunKAISTData glog ${OpenCV_LIBS} TinyGrapeKit ${PROJECT_NAME})

add_executable(RunSimData example/RunSimData.cpp)
target_link_libraries(RunSimData
    ${catkin_LIBRARIES}
    ${PROJECT_NAME} glog ${OpenCV_LIBS} TinyGrapeKit 
)

add_executable(TestVisualWheelCircleSim example/TestVisualWheelCircleSim.cpp)
target_link_libraries(TestVisualWheelCircleSim
    ${catkin_LIBRARIES}
    ${PROJECT_NAME} glog ${OpenCV_LIBS} TinyGrapeKit 
)
