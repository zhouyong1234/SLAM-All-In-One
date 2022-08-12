cmake_minimum_required(VERSION 2.8.3)
project(image_alignment)
add_definitions(-DPC_VERSION)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g -ggdb")

find_package(Protobuf REQUIRED)
find_package(nanomsg CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS interface/image.proto interface/rovio.proto)

include_directories(
        include
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR})

add_library(pb_cpp SHARED
        ${PROTO_SRCS}
        )
target_link_libraries(pb_cpp
        ${PROTOBUF_LIBRARY}
        )

add_executable(image_alignment app/test_image_alignment.cpp)
target_link_libraries(image_alignment ${OpenCV_LIBS})

add_executable(imageAlignment app/test_ImageAlignment.cpp)
target_link_libraries(imageAlignment ${OpenCV_LIBS})

add_executable(test_replay app/test_replay.cpp)
target_link_libraries(test_replay ${OpenCV_LIBS} ${PROTOBUF_LIBRARY} pb_cpp nanomsg)


add_executable(test_replay_advance app/test_replay_advance.cpp)
target_link_libraries(test_replay_advance ${OpenCV_LIBS} ${PROTOBUF_LIBRARY} pb_cpp nanomsg)

add_executable(test_opticalflow app/test_opticalflow.cpp)
target_link_libraries(test_opticalflow ${OpenCV_LIBS} ${PROTOBUF_LIBRARY} pb_cpp nanomsg)