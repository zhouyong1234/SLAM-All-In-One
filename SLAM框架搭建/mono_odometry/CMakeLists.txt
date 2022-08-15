cmake_minimum_required(VERSION 3.8)
project(mono_odometry)
set(CMAKE_CXX_STANDARD 11)


#SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)
# ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)


# ============ Add something ===========
file(GLOB_RECURSE SOURCES src/*.cpp lib/*.cpp )
file(GLOB_RECURSE HEADERS include/*.hpp lib/*.hpp include/*.h lib/*.h)

# ============ Find OpenCV =============
find_package(OpenCV 3.0)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
else()
    message(STATUS "Find OpenCV.")
    INCLUDE_DIRECTORIES(  ${OpenCV_INCLUDE_DIRS}  )
endif ()

link_directories(${CMAKE_INSTALL_PREFIX}/lib)
include_directories(${CMAKE_INSTALL_PREFIX}/include)

set(BUILD_SHARED_LIBS OFF)


add_executable(mono_vo  ${SOURCES} ${HEADERS})
target_link_libraries(mono_vo ${OpenCV_LIBS} thor)