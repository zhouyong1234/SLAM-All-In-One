cmake_minimum_required(VERSION 2.8)
project(MSCKF_MINE)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)



find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories("/usr/include/eigen3")
#find_package(Eigen3 3.1.0 REQUIRED)

# Sophus
#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/test
${PROJECT_SOURCE_DIR}/test/optical_flow_test
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/msckf.cc
    src/config.cc
    src/data_reader.cc
    src/converter.cc
    src/camera.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/frame.cc

    

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CERES_LIBRARIES}
)

# Build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(read_csv_file test/read_csv_file.cpp )
target_link_libraries(read_csv_file ${PROJECT_NAME})

add_executable(load_imu_camera_data test/load_imu_camera_data.cpp )
target_link_libraries(load_imu_camera_data ${PROJECT_NAME})

add_executable(camera_test test/camera_test.cpp )
target_link_libraries(camera_test ${PROJECT_NAME})


add_executable(msckf_construct_test test/msckf_construct_test.cpp )
target_link_libraries(msckf_construct_test ${PROJECT_NAME})

add_executable(eigen_function_test test/eigen_function_test.cpp )
target_link_libraries(eigen_function_test ${PROJECT_NAME})

add_executable(orb_feature_extract test/orb_feature_extract.cpp )
target_link_libraries(orb_feature_extract ${PROJECT_NAME})

add_executable(orb_feature_match_test test/orb_feature_match_test.cpp )
target_link_libraries(orb_feature_match_test ${PROJECT_NAME})

add_executable(frame_and_orb_test test/frame_and_orb_test.cpp )
target_link_libraries(frame_and_orb_test ${PROJECT_NAME})

add_executable(msckf_test_v0 test/msckf_test_v0.cpp )
target_link_libraries(msckf_test_v0 ${PROJECT_NAME})

add_executable(optical_flow_tracking test/optical_flow_test/optical_flow_tracking.cpp )
target_link_libraries(optical_flow_tracking ${PROJECT_NAME})

add_executable(optical_flow_tracking_frame2frame test/optical_flow_test/optical_flow_tracking_frame2frame.cpp )
target_link_libraries(optical_flow_tracking_frame2frame ${PROJECT_NAME}) 

add_executable(asl_dataset_to_mars test/asl_dataset_to_mars.cpp )
target_link_libraries(asl_dataset_to_mars ${PROJECT_NAME})

add_executable(optical_flow_tracking_mask test/optical_flow_test/optical_flow_tracking_mask.cpp )
target_link_libraries(optical_flow_tracking_mask ${PROJECT_NAME})


