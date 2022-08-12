cmake_minimum_required(VERSION 3.1.0)
project(d435i_sdk)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_INSTALL_PREFIX "../../env/")


if (APPLE)
find_package(OpenCV 4.0 REQUIRED)
else()
set(OpenCV_DIR "/home/kdq/packages/opencv4/opencv/build")
find_package(OpenCV 4.0 REQUIRED)
message(STATUS "OpenCV_Path = ${OpenCV_INCLUDE_DIRS}")
endif()

find_package(realsense2 REQUIRED)
message(STATUS "realsense2_INCLUDE_DIR = ${realsense2_INCLUDE_DIR}")
message(STATUS "realsense2_LIBRARY = ${realsense2_LIBRARY}")


include_directories(include ${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIR})

add_library(d435i SHARED src/d435i.cpp include/d435iConfig.hpp)
target_link_libraries(d435i ${realsense2_LIBRARY} ${OpenCV_LIBS})

add_executable(test_infrared app/test_infrared.cpp)
target_link_libraries(test_infrared d435i)

add_executable(test_all_stream app/test_all_stream.cpp)
target_link_libraries(test_all_stream d435i)

add_executable(d435i_calibr app/d435i_intrinisic_calibr.cpp)
target_link_libraries(d435i_calibr d435i)

add_executable(test_detect_aruco app/test_detect_aruco.cpp)
target_link_libraries(test_detect_aruco d435i)

add_executable(test_charuco_board app/test_charuco_board.cpp)
target_link_libraries(test_charuco_board d435i)

INSTALL(TARGETS d435i  
        LIBRARY DESTINATION lib/d435i)

INSTALL(DIRECTORY include/ DESTINATION include/d435i) 