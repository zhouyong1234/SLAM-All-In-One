cmake_minimum_required(VERSION 2.8.3)
project(viz_scene)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")
set(CMAKE_INSTALL_PREFIX "../../env/")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Ceres REQUIRED)

message(STATUS  ${OpenCV_INCLUDE_DIRS})
message(STATUS ${EIGEN3_INCLUDE_DIR})
include_directories(inc ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_library(camera_models 
    src/camodocal/chessboard/Chessboard.cc
    src/camodocal/calib/CameraCalibration.cc
    src/camodocal/camera_models/Camera.cc
    src/camodocal/camera_models/CameraFactory.cc
    src/camodocal/camera_models/CostFunctionFactory.cc
    src/camodocal/camera_models/PinholeCamera.cc
    src/camodocal/camera_models/PinholeFullCamera.cc
    src/camodocal/camera_models/CataCamera.cc
    src/camodocal/camera_models/EquidistantCamera.cc
    src/camodocal/camera_models/ScaramuzzaCamera.cc
    src/camodocal/sparse_graph/Transform.cc
    src/camodocal/gpl/gpl.cc
    src/camodocal/gpl/EigenQuaternionParameterization.cc)
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
add_library(vizscene STATIC src/viz_scene/viz_scene.cpp
            src/imu_motion/imu_motion.cpp
            src/camera_project/camera_project.cpp)
target_link_libraries(vizscene camera_models ${OpenCV_LIBS} -lpthread -lm)

add_executable(testCameraMotion src/testCameraMotion.cpp)
target_link_libraries(testCameraMotion vizscene)



#INSTALL(TARGETS hello hello_static
 #      LIBRARY DESTINATION include/lib
 #      ARCHIVE DESTINATION include/lib)
INSTALL(TARGETS vizscene 
        ARCHIVE DESTINATION lib/viz_scene)
INSTALL(TARGETS camera_models 
        ARCHIVE DESTINATION lib/viz_scene)
INSTALL(DIRECTORY inc/ DESTINATION include/viz_scene) 
