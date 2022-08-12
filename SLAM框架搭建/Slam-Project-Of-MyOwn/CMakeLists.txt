#required the minimum version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 3.12.2 )

# project name
PROJECT( slam )

# c++11 or higher version
ADD_DEFINITIONS( -std=c++14 )

# option of turning on the log or not
OPTION( ONLOG "log on or off" OFF )


IF( ONLOG )
	ADD_DEFINITIONS( -DTERMINAL_LOG )
ENDIF( ONLOG )


# set the g2o vendored ceres
set(G2O_USE_VENDORED_CERES ON CACHE BOOL "Use vendored Ceres from g2o/EXTERNAL/ceres")


# set the output directory of the libs
SET( LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib" )

#
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# find g2o library
FIND_PACKAGE( G2O REQUIRED )
IF( G2O_FOUND )
	INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} )
	MESSAGE( STATUS "G2O lib is found: ${G2O_INCLUDE_DIR}" )
ENDIF(G2O_FOUND)

# find CSparse
FIND_PACKAGE( CSparse REQUIRED )

# find required eigen
FIND_PACKAGE( Eigen3 )

# directory of CSparse
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )

# directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# find required opencv
FIND_PACKAGE( OpenCV REQUIRED )
MESSAGE( STATUS "Opencv lib is found: " )

# directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
MESSAGE( STATUS "Opencv Include Directory: ${OpenCV_INCLUDE_DIRS}" )

# add the source files
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src slam_src )
MESSAGE( STATUS "Find Source Files In: ${PROJECT_SOURCE_DIR}/src" )

# headers of the source files
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include" )
MESSAGE( STATUS "Find Header Files In: ${PROJECT_SOURCE_DIR}/include" )

# directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
MESSAGE( STATUS "Opencv Lib Directory: ${OpenCV_LIBRARY_DIRS}" )

SET( G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )


# generate the lib files
ADD_LIBRARY( slam SHARED ${slam_src} )

ADD_LIBRARY( slam_static STATIC ${slam_src} )

# link libraries
TARGET_LINK_LIBRARIES( slam ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( slam_static ${OpenCV_LIBS} )

TARGET_LINK_LIBRARIES( slam ${G2O_LIBS} )
TARGET_LINK_LIBRARIES( slam_static ${G2O_LIBS} )

SET_TARGET_PROPERTIES( slam_static PROPERTIES OUTPUT_NAME "slam" )
