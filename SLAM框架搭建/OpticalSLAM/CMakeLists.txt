cmake_minimum_required(VERSION 2.8)
project(OpticalSLAM)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(Pangolin 0.4 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} 
"./include" 
"/usr/local/include/eigen3")

add_executable(kitti_slam kitti_main.cpp)
target_link_libraries(kitti_slam ${Pangolin_LIBRARIES} ${OpenCV_LIBS} OpticalSLAM)

add_executable(test_corner test_corner.cpp)
target_link_libraries(test_corner ${Pangolin_LIBRARIES} ${OpenCV_LIBS} OpticalSLAM)

add_executable(test_equalizeHist test_equalizeHist.cpp)
target_link_libraries(test_equalizeHist ${Pangolin_LIBRARIES} ${OpenCV_LIBS} OpticalSLAM)

add_executable(test_pyramid test_pyramid.cpp)
target_link_libraries(test_pyramid ${Pangolin_LIBRARIES} ${OpenCV_LIBS} OpticalSLAM)

add_subdirectory(src)