cmake_minimum_required(VERSION 2.8.3)
project(depth_filter)
add_definitions(-DVERBOOSE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 ${SSE_FLAGS} -msse4 -lgsl -pthread")
get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
set(VIZ_SCENE_INCLUDE "${PARENT_DIRECTORY}/env/include/viz_scene")
set(VIZ_SCENE_LIBS "${PARENT_DIRECTORY}/env/lib/viz_scene/libvizscene.a;${PARENT_DIRECTORY}/env/lib/viz_scene/libcamera_models.a")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
  ${Boost_INCLUDE_DIRS}
  ${VIZ_SCENE_INCLUDE}
)

add_executable(depth_filter 
  app/test1.cpp
  src/DepthFilter.cpp
)
target_link_libraries(depth_filter  
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${VIZ_SCENE_LIBS}
)

add_executable(testDf
        app/testDF.cpp
        src/DepthFilter.cpp
        )