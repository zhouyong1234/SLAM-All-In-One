cmake_minimum_required(VERSION 3.0.2)
project(live_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	tf_conversions
	std_msgs
	sensor_msgs
	geometry_msgs
	nav_msgs
	message_generation
	visualization_msgs
	cv_bridge
	pcl_conversions
)

add_message_files(
	DIRECTORY msg
	FILES
	cloud_info.msg
)

add_service_files(
	DIRECTORY srv
	FILES
	save_map.srv
)

generate_messages(
		DEPENDENCIES
		geometry_msgs
		std_msgs
		nav_msgs
		sensor_msgs
)


find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
	INCLUDE_DIRS include
	DEPENDS PCL OpenCV GTSAM

	CATKIN_DEPENDS
	std_msgs
	nav_msgs
	geometry_msgs
	sensor_msgs
	message_runtime
	message_generation
	visualization_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
	${OpenCV_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_pointProjection src/pointProjection.cpp)
add_dependencies(${PROJECT_NAME}_pointProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_pointProjection PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_pointProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})

add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)