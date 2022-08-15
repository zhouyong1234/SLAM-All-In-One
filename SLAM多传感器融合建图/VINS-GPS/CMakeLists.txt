cmake_minimum_required(VERSION 3.5)
project(VINS_GPS)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_SOURCE_DIR}/camera_model/include)
include_directories(${CMAKE_SOURCE_DIR}/msgg)

include_directories(
        /usr/local/include/
        /usr/local/include/opencv
)
set(OPENCV_LIB_DIR "/usr/local/lib")
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
include_directories(
        /usr/include/eigen3
)
#find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(VINS_GPS main.cpp
        factor/imu_factor.h
        factor/gps_factor.h
        factor/integration_base.h
        factor/pose_local_parameterization.cpp
        factor/pose_local_parameterization.h
        factor/projection_factor.h
        factor/projection_factor.cpp
        factor/projection_td_factor.h
        factor/projection_td_factor.cpp
        initial/initial_alignment.h
        initial/initial_aligment.cpp
        initial/initial_ex_rotation.h
        initial/initial_ex_rotation.cpp
        initial/initial_sfm.h
        initial/initial_sfm.cpp
        initial/solve_5pts.h
        initial/solve_5pts.cpp
        utility/tic_toc.h
        utility/utility.h
        utility/utility.cpp
        feature_manager.cpp
        feature_manager.h
        feature_tracker.h
	Coosys.h
        feature_tracker.cpp parameters.cpp parameters.h estimator.h estimator.cpp )

target_link_libraries(VINS_GPS
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/camera_model/build/libcamera_model.a)
