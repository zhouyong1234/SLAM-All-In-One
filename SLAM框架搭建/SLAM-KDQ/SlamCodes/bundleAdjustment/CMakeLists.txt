cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_STANDARD 14)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_module)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(CSparse REQUIRED)
find_package(fmt REQUIRED)
find_package(SuiteSparse REQUIRED)

include_directories(include/ba
        ${EIGEN3_INCLUDE_DIR}
        ${CSPARSE_INCLUDE_DIR}

        ${G2O_INCLUDE_DIRS})
SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core g2o_solver_cholmod cholmod g2o_types_sba g2o_types_slam3d fmt::fmt cxsparse)
#add_library(SHARED bag2o src/BundleAdjustmentByG2O.cpp)
#target_link_libraries(bag2o ${G2O_LIBS})

add_executable(test_g2o src/test_g2o.cpp)
target_link_libraries(test_g2o ${G2O_LIBS})
