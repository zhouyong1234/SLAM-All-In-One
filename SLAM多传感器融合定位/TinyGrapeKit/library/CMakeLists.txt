cmake_minimum_required(VERSION 2.8.8)
project(TinyGrapeKit)

# Set C++14
add_compile_options(-std=c++14)

# Set cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Third party dependencies
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Glog  REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}/opencv4)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(include)
# Make library
file(GLOB_RECURSE ALL_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/*.h
)
file(GLOB_RECURSE ALL_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${ALL_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES})

# install
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/TGK
)
