cmake_minimum_required(VERSION 3.10)

include("cmake_modules/PreProjectWorkarounds.cmake")

project(basalt-headers)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

if (NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  else()
    message(STATUS "Dind't find ccache")
  endif()
else()
  message(STATUS "Compiler launcher already set. Not configuring ccache.")
  message(STATUS "CMAKE_C_COMPILER_LAUNCHER: ${CMAKE_C_COMPILER_LAUNCHER}")
  message(STATUS "CMAKE_CXX_COMPILER_LAUNCHER: ${CMAKE_CXX_COMPILER_LAUNCHER}")
endif()

IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release)
ENDIF()

IF(NOT CXX_MARCH)
   SET(CXX_MARCH native)
ENDIF()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# clang-specific compile flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(BASALT_HEADERS_CXX_FLAGS "${BASALT_HEADERS_CXX_FLAGS} -Wno-exceptions -fcolor-diagnostics -frelaxed-template-template-args -Wno-error=deprecated-declarations")

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10)
    # These are disabled to avoid lot's of warnings in Eigen code with clang 10
    set(BASALT_HEADERS_CXX_FLAGS "${BASALT_HEADERS_CXX_FLAGS} -Wno-error=misleading-indentation")
  endif()
else()
  set(BASALT_HEADERS_CXX_FLAGS "${BASALT_HEADERS_CXX_FLAGS} -Wno-error=maybe-uninitialized")
endif()


set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
SET(CMAKE_CXX_FLAGS "-Wall -Werror -Wextra -march=${CXX_MARCH} ${CMAKE_CXX_FLAGS} ${BASALT_HEADERS_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
SET(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
SET(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
set(CMAKE_CXX_FLAGS_SANITIZERDEBUG "-O0 -g0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -fno-omit-frame-pointer -fsanitize=address,undefined")
set(CMAKE_CXX_FLAGS_SANITIZERRELWITHDEBINFO "-O3 -g0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -fno-omit-frame-pointer -fsanitize=address,undefined")


set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen")
set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
find_package(Eigen3 3.3.7 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")

include_directories(thirdparty/Sophus)
include_directories(thirdparty/cereal/include)
include_directories(include)

enable_testing()
add_subdirectory(test)



