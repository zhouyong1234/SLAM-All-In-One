cmake_minimum_required(VERSION 2.8.3)
project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories("include")

add_executable(Calibration 
    srcc/intrinsic_calib.cc
    srcc/chessboard/Chessboard.cc
    srcc/calib/CameraCalibration.cc
    srcc/camera_models/Camera.cc
    srcc/camera_models/CameraFactory.cc
    srcc/camera_models/CostFunctionFactory.cc
    srcc/camera_models/PinholeCamera.cc
    srcc/camera_models/CataCamera.cc
    srcc/camera_models/EquidistantCamera.cc
    srcc/camera_models/ScaramuzzaCamera.cc
    srcc/sparse_graph/Transform.cc
    srcc/gpl/gpl.cc
    srcc/gpl/EigenQuaternionParameterization.cc)

add_library(camera_model STATIC
        srcc/chessboard/Chessboard.cc
        srcc/calib/CameraCalibration.cc
        srcc/camera_models/Camera.cc
        srcc/camera_models/CameraFactory.cc
        srcc/camera_models/CostFunctionFactory.cc
        srcc/camera_models/PinholeCamera.cc
        srcc/camera_models/CataCamera.cc
        srcc/camera_models/EquidistantCamera.cc
        srcc/camera_models/ScaramuzzaCamera.cc
        srcc/sparse_graph/Transform.cc
        srcc/gpl/gpl.cc
        srcc/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(Calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
