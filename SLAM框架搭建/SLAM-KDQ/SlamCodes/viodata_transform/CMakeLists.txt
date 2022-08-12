cmake_minimum_required(VERSION 2.8.3)
project(viodata_transform)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
set(CMAKE_INSTALL_PREFIX "../../env/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
message(STATUS ${PARENT_DIRECTORY})
set(DATA_INTERFACE_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/data_interface")
set(DATA_INTERFACE_LIBS "${PARENT_DIRECTORY}/env/lib/data_interface/libZeroZeroImu.a")

find_package(OpenCV REQUIRED)
find_package(nanomsg CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(inc 
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${DATA_INTERFACE_INCLUDE_DIR}
  )

add_executable(rec_to_file 
  
  src/transformRecToFile.cpp)
target_link_libraries(rec_to_file 
  ${OpenCV_LIBS}
  ${PROTOBUF_LIBRARIES}
  ${DATA_INTERFACE_LIBS}
  nanomsg)