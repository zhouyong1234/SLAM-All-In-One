cmake_minimum_required (VERSION 2.6)
project(zz_replay)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
get_filename_component(GRANDFATHER_DIRECTORY ${PARENT_DIRECTORY} DIRECTORY)
get_filename_component(ENV_PATH ${GRANDFATHER_DIRECTORY} DIRECTORY)
message(STATUS ${ENV_PATH})
set(REPLAYKIT_INCLUDE_DIR "${ENV_PATH}/env/include/replaykit")
set(REPLAYKIT_LIBS "${ENV_PATH}/env/lib/replaykit/libreplaykit.a;${ENV_PATH}/env/lib/replaykit/libreplay_proto.a")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf
    cv_bridge
    rosbag
)

find_package(Eigen3)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nanomsg CONFIG REQUIRED)

catkin_package()

if(NOT PROTOBUF_FOUND)
    message (FATAL_ERROR "Cannot find Protobuf")
endif()

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS 
    interface/rovio.proto)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${DATA_INTERFACE_INCLUDE_DIR}
    ${REPLAYKIT_INCLUDE_DIR}
)

add_library(pb_cpp SHARED
    ${PROTO_SRCS}
)
target_link_libraries(pb_cpp
    ${PROTOBUF_LIBRARY}
)
add_executable(zz_replay
    src/main.cpp
)
target_link_libraries(zz_replay 
    pb_cpp 
    ${OpenCV_LIBS}     
    ${PROTOBUF_LIBRARIES}
    nanomsg
    ${catkin_LIBRARIES}
) 

add_executable(replaykit src/replaykit.cpp)
target_link_libraries(replaykit  
    pb_cpp
    nanomsg
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}     
    ${PROTOBUF_LIBRARIES}
    ${REPLAYKIT_LIBS}
)