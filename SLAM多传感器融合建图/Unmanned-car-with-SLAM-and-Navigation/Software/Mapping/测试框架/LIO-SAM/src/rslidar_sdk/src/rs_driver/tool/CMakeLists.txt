cmake_minimum_required(VERSION 3.5)
project(rs_driver_viewer)
message(=============================================================)
message("-- Ready to compile tools")
message(=============================================================)
include_directories(${DRIVER_INCLUDE_DIRS})
set(CMAKE_BUILD_TYPE Release)
if(WIN32)
  cmake_policy(SET CMP0074 NEW)
  set(OPENNI_ROOT "C:\\Program Files\\OpenNI2")
  set(OPENNI_LIBRARY "${OPENNI_ROOT}\\Lib\\OpenNI2.lib")
  set(OPENNI_INCLUDE_DIRS "${OPENNI_ROOT}\\Include\\")
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Release)
  file(COPY ${OPENNI_ROOT}\\Redist\\OpenNI2.dll DESTINATION ${PROJECT_BINARY_DIR}\\Debug)
endif(WIN32)
find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
if(PCL_FOUND)
add_executable(rs_driver_viewer
               rs_driver_viewer.cpp
              )
target_link_libraries(rs_driver_viewer
                    ${EXTERNAL_LIBS}    
                    ${PCL_LIBRARIES}   
)
else()
message("PCL Not found! Can not compile rs_driver_viewer!")
endif()

install(TARGETS rs_driver_viewer
        RUNTIME DESTINATION /usr/bin
)     
