SET(PROJECT_NAME PhotometricDatasetCalibration)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(EXECUTABLE_OUTPUT_PATH bin)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibZip 1.1.1 REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS_RELEASE} ${SSE_FLAGS} -std=c++0x"
)

include_directories(
	${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${LIBZIP_INCLUDE_DIR_ZIP}
        ${LIBZIP_INCLUDE_DIR_ZIPCONF}
)


add_executable(responseCalib src/main_responseCalib.cpp src/FOVUndistorter.cpp src/PhotometricUndistorter.cpp)
target_link_libraries(responseCalib ${OpenCV_LIBS} ${LIBZIP_LIBRARY})

add_executable(playDataset src/main_playbackDataset.cpp src/FOVUndistorter.cpp src/PhotometricUndistorter.cpp)
target_link_libraries(playDataset ${OpenCV_LIBS} ${LIBZIP_LIBRARY})






SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
find_package(aruco)

IF(aruco_FOUND)
	add_executable(vignetteCalib src/main_vignetteCalib.cpp src/FOVUndistorter.cpp src/PhotometricUndistorter.cpp)
	target_link_libraries(vignetteCalib ${OpenCV_LIBS} ${aruco_LIBS} ${LIBZIP_LIBRARY})
ELSE()
	message("================ aruco not found. not compiling vignetteCalib. ========================")
ENDIF()















