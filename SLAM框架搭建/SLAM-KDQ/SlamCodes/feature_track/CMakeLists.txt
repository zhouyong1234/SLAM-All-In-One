cmake_minimum_required(VERSION 2.8.3)
project(feature_track)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
set(EXECUTABLE_OUTPUT_PATH "bin")
get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
message(STATUS ${PARENT_DIRECTORY})
set(DATA_INTERFACE_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/data_interface")
set(REPLAYKIT_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/replaykit")
set(REPLAYKIT_LIBS "${PARENT_DIRECTORY}/env/lib/replaykit/libreplaykit.a;${PARENT_DIRECTORY}/env/lib/replaykit/libreplay_proto.a")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(inc/feature_track
  inc/interface
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${DATA_INTERFACE_INCLUDE_DIR}
  ${REPLAYKIT_INCLUDE_DIR})

add_executable(test 
  src/main.cpp
  src/optiflow_track/vins_optitrack.cpp)
target_link_libraries(test 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES}
  ${REPLAYKIT_LIBS}
)

  add_executable(gftt_track
  src/gftt_test.cpp
  src/optiflow_track/vins_optitrack.cpp)

  target_link_libraries(gftt_track
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES}
  )


  add_executable(replayFalcon
  src/falcon_test.cpp
  src/optiflow_track/vins_optitrack.cpp)

  target_link_libraries(replayFalcon
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES}
  ${DATA_INTERFACE_LIBS}
  ${REPLAYKIT_LIBS})