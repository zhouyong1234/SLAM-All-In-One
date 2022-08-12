cmake_minimum_required(VERSION 2.8)
project(gauss_filter)
set(CMAKE_INSTALL_PREFIX "../../env/")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -lgsl")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIRS})
add_executable(test_gauss1 app/test1.cpp)
target_link_libraries(test_gauss1 gsl gslcblas)
add_executable(test_gauss2 app/test2.cpp)
target_link_libraries(test_gauss2 gsl gslcblas)

add_executable(test_gauss3 app/test3.cpp)
target_link_libraries(test_gauss3 gsl gslcblas)

INSTALL(DIRECTORY include/ DESTINATION include/GaussFilter)
