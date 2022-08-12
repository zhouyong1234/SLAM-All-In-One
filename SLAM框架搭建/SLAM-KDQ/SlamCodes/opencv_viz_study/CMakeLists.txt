cmake_minimum_required(VERSION 2.8.3)
project(viz_lesson)
find_package(OpenCV REQUIRED)
message(STATUS ${OpenCV_INCLUDE_DIRS})
include_directories(inc ${OpenCV_DIRS})


add_executable(viz_lesson1 src/viz_lesson1_window.cpp)
target_link_libraries(viz_lesson1 ${OpenCV_LIBS})

add_executable(viz_lesson2 src/viz_lesson2_widget.cpp)
target_link_libraries(viz_lesson2 ${OpenCV_LIBS})

add_executable(viz_lesson3 src/viz_lesson3_camera.cpp)
target_link_libraries(viz_lesson3 ${OpenCV_LIBS})