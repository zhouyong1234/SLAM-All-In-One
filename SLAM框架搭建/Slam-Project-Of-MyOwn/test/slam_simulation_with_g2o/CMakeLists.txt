CMAKE_MINIMUM_REQUIRED( VERSION 3.12.2 )

PROJECT( slamSimulationWithG2O )

ADD_DEFINITIONS( -std=c++14 )

SET( EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin" )
MESSAGE( STATUS "executable files path: " ${PROJECT_BINARY_DIR}/bin )


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)
MESSAGE( STATUS "cmake moudules path : ${CMAKE_MODULE_PATH}" )

# find g2o library
FIND_PACKAGE( G2O REQUIRED )
IF( G2O_FOUND )
        INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} )
        MESSAGE( STATUS "G2O lib is found: ${G2O_INCLUDE_DIR}" )
ENDIF(G2O_FOUND)

# find CSparse
FIND_PACKAGE( CSparse REQUIRED )

#find required eigen
FIND_PACKAGE( Eigen3 )

# directory of CSparse
INCLUDE_DIRECTORIES( ${CSPARSE_INCLUDE_DIR} )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

# find required pcl
#FIND_PACKAGE( PCL REQUIRED )


# directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# directory of pcl headers
#INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )

# pcl definitions
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR} test_src )
MESSAGE( STATUS "source path: " ${PROJECT_SOURCE_DIR} )

INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/../../include" )
MESSAGE( STATUS "include path: " ${PROJECT_SOURCE_DIR}/../../include )

LINK_DIRECTORIES( "${PROJECT_SOURCE_DIR}/../../build/lib/" )
MESSAGE( STATUS "lib include path: " ${PROJECT_SOURCE_DIR}/../../build/lib )


# directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

SET( G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )


ADD_EXECUTABLE( slamSimulationWithG2O ${test_src} )

TARGET_LINK_LIBRARIES( slamSimulationWithG2O slam.a )

TARGET_LINK_LIBRARIES( slamSimulationWithG2O ${OpenCV_LIBS} )

TARGET_LINK_LIBRARIES( slamSimulationWithG2O ${G2O_LIBS} )

#TARGET_LINK_LIBRARIES( slamSimulationWithG2O ${PCL_LIBRARIES} )

