cmake_minimum_required(VERSION 2.6)
project(data_interface)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -ggdb")
get_filename_component(PARENT_DIRECTORY ${PROJECT_SOURCE_DIR} DIRECTORY)
message(STATUS ${PARENT_DIRECTORY})
set(REPLAYKIT_INCLUDE_DIR "${PARENT_DIRECTORY}/env/include/replaykit")
set(REPLAYKIT_LIBS "${PARENT_DIRECTORY}/env/lib/replaykit/libreplaykit.a;${PARENT_DIRECTORY}/env/lib/replaykit/libreplay_proto.a")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")
set(CMAKE_INSTALL_PREFIX "../../env/")
set(EXECUTABLE_OUTPUT_PATH "../bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "../lib")
#构建时候用add_custom_target
add_custom_target(create_lib ALL COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

#配置的时候用file
# file(MAKE_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
# file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX})
find_package(OpenCV REQUIRED)
find_package(nanomsg CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)


include_directories(
  include
  ${PROTOBUF_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${REPLAYKIT_INCLUDE_DIR})


add_executable(test 
  src/main.cpp)
target_link_libraries(test 
  ${OpenCV_LIBS}
  ${REPLAYKIT_LIBS}
  ${PROTOBUF_LIBRARIES}
  nanomsg)

INSTALL(DIRECTORY include/zz_vio_interface DESTINATION include/${PROJECT_NAME})
