cmake_minimum_required(VERSION 2.8.3)
project(code_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC -fopenmp")

find_package(catkin REQUIRED
    roscpp
    std_msgs
    )

#set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES polynomial cv_utils pnp
    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
     )
include_directories("include")

add_library(polynomial STATIC
    src/math_utils/Polynomial.cpp)
target_link_libraries(polynomial ${Boost_LIBRARIES} )

add_library(pnp
    src/cv_utils/dlt/dlt.cpp
    src/cv_utils/pnp/pnp.cpp
    src/cv_utils/pnp/linearpnp.cpp
    src/cv_utils/pnp/nonlinearpnp.cpp)
target_link_libraries(pnp ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${CERES_LIBRARIES})

add_library(cv_utils STATIC
    src/cv_utils.cc
    )
target_link_libraries(cv_utils ${Boost_LIBRARIES}  ${OpenCV_LIBS} )

add_executable(matIO_test   src/mat_io_test.cpp )
target_link_libraries(matIO_test dw ${OpenCV_LIBS})

add_executable(sumpixel_test   src/sumpixel_test.cpp )
target_link_libraries(sumpixel_test dw ${OpenCV_LIBS})
