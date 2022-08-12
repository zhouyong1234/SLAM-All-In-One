cmake_minimum_required(VERSION 2.8.3)
project(image_grayValue_histogram)
find_package(OpenCV REQUIRED)
include_directories(include 
  ${OpenCV_INCLUDE_DIRS})
add_executable(test1   
  app/test1.cpp
        )
target_link_libraries(test1 
  ${OpenCV_LIBS})

add_executable(test2  
  app/test2.cpp
        )
target_link_libraries(test2 
  ${OpenCV_LIBS})
