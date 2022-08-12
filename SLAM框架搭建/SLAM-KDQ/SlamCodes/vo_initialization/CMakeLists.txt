cmake_minimum_required(VERSION 2.8.3)
project(vo_initialization)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
message(STATUS ${PARENT_DIRECTORY})
set(VIZ_SCENE_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/viz_scene")
set(VIZ_SCENE_LIBS "${PARENT_DIRECTORY}/env/lib/viz_scene/libvizscene.a")
set(VIZ_CAMERA_LIBS "${PARENT_DIRECTORY}/env/lib/viz_scene/libcamera_models.a")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(inc 
  ${EIGEN3_INCLUDE_DIR}                    
  ${OpenCV_INCLUDE_DIRS}
  ${VIZ_SCENE_INCLUDE_DIR})
add_executable(vo_ini src/main.cpp
               src/Frame/frame.cpp
               src/Initialization/Initializator.cpp)
target_link_libraries(vo_ini
  ${OpenCV_LIBS}
  ${VIZ_SCENE_LIBS}
  ${VIZ_CAMERA_LIBS}
  -lpthread -lm)