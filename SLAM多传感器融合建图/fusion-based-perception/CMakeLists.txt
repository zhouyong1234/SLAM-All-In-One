cmake_minimum_required(VERSION 3.5)
project(kit_perception)
add_compile_options(-std=c++14 -g)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
            roscpp
            sensor_msgs
            cv_bridge
        )

catkin_package(
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        cv_bridge
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )

file(GLOB _SRCS "fusion/*.cpp")

add_library(kit_perception_fusion ${_SRCS})
target_link_libraries(kit_perception_fusion PUBLIC
	${OpenCV_LIBS}
	yaml-cpp)

set(EZCFG_STRUCT_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/input_data_type.h)
add_subdirectory(easy_config)
add_subdirectory(backward-cpp)

add_executable(sensor_fusion_node
    fusion_node.cpp
    frame_build.cpp
    visualization.cpp
    ${BACKWARD_ENABLE})
add_backward(sensor_fusion_node)

target_link_libraries(sensor_fusion_node PRIVATE
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    kit_perception_fusion
    ezcfg::ezcfg)

target_include_directories(sensor_fusion_node PRIVATE ${Boost_INCLUDE_DIRS})
