cmake_minimum_required(VERSION 2.8)
project(cv_mat_lesson)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_mat_data_storage_method test/test_mat_data.cpp)
target_link_libraries(test_mat_data_storage_method 
  ${OpenCV_LIBS})