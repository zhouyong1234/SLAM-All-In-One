cmake_minimum_required(VERSION 2.8)
project(bspline)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -lgsl")
set(SPLINE_INCLUDE "/usr/local/include/SPLINTER/")
set(SPLINE_LIB "/usr/local/lib/libsplinter-3-0.so")
set(CMAKE_INSTALL_PREFIX "../../env/")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable(bSpline app/test.cpp)
target_link_libraries(bSpline gsl gslcblas)

add_executable(bSpline1 app/test1.cpp)
target_link_libraries(bSpline1 gsl gslcblas)

add_executable(gslSpline1 app/gsl_spline1.cpp)
target_link_libraries(gslSpline1 gsl gslcblas)

include_directories(${SPLINE_INCLUDE} include)
add_executable(spline app/spline.cpp)
target_link_libraries(spline ${SPLINE_LIB})

add_executable(spline1 app/spline1.cpp)
target_link_libraries(spline1 ${SPLINE_LIB})

add_executable(spline2 app/spline2.cpp)
target_link_libraries(spline2 ${SPLINE_LIB})

add_executable(spline3 app/spline3.cpp)
target_link_libraries(spline3 ${SPLINE_LIB})

add_executable(spline4 app/spline4.cpp)
target_link_libraries(spline4 ${SPLINE_LIB})

INSTALL(DIRECTORY include/ DESTINATION include/BSplineX)
