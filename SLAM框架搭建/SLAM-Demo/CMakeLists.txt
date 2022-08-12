cmake_minimum_required(VERSION 2.8)
project( TESTFILES )
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

message(STATUS "[INFO]:opencv is searching....")
find_package( OpenCV REQUIRED)
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories(
    "/usr/include/eigen3"
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    # ${PROJECT_SOURCE_DIR}/include/DataPretreat
    # ${PROJECT_SOURCE_DIR}/include/FrontEnd
    # ${PROJECT_SOURCE_DIR}/include/BackEnd
    # ${PROJECT_SOURCE_DIR}/include/LoopClosing
    # ${PROJECT_SOURCE_DIR}/include/Viewer
    # ${PROJECT_SOURCE_DIR}/include/Tools
    ${CSPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/DataPretreat/Frame.cc
    src/DataPretreat/DepthMap.cc
    src/FrontEnd/Tracking.cc
    src/FrontEnd/FeatureManager.cc
    src/FrontEnd/EpipolarLine.cc
    src/FrontEnd/PoseSolver.cc
    src/BackEnd/Optimization.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${CSPARSE_LIBRARY}
g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(main app/main.cpp)
target_link_libraries( main ${PROJECT_NAME})
