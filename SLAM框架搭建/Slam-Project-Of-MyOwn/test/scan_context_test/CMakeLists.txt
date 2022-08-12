CMAKE_MINIMUM_REQUIRED( VERSION 3.21.1 )

ADD_DEFINITIONS( -std=c++14 )

SET( EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin" )

#find required eigen
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

#find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

#directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


# change this directory for required source file
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/scan_context_test test_src )

INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/../include" )

LINK_DIRECTORIES( "${PROJECT_SOURCE_DIR}/../build/lib/" )

#directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

# executable file name
ADD_EXECUTABLE( scanContext ${test_src} )

TARGET_LINK_LIBRARIES( scanContext slam.a )

TARGET_LINK_LIBRARIES( scanContext ${OpenCV_LIBS} )
