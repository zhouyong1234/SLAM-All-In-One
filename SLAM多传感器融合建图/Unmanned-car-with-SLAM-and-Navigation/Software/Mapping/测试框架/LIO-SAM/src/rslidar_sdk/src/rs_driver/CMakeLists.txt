cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
if(WIN32)
  cmake_policy(SET CMP0074 NEW)
endif(WIN32)
project(rs_driver VERSION 1.3.0)

#=============================
#  Compile Demos&Tools
#=============================
option(COMPILE_DEMOS "Build rs_driver demos" OFF)
option(COMPILE_TOOLS "Build point cloud visualization tool" OFF)

#=============================
#  Compile Demos&Tools
#=============================
option(ENABLE_TRANSFORM "Enable transform functions" OFF)

#========================
#  Project setup
#========================
set(CMAKE_BUILD_TYPE Release)

#========================
#  Platform cross setup
#========================
if(MSVC)
  set(COMPILER "MSVC Compiler")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
  set(CompilerFlags
    CMAKE_CXX_FLAGS                       CMAKE_C_FLAGS
    CMAKE_CXX_FLAGS_DEBUG                 CMAKE_C_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE               CMAKE_C_FLAGS_RELEASE
	CMAKE_CXX_FLAGS_MINSIZEREL            CMAKE_C_FLAGS_MINSIZEREL
	CMAKE_CXX_FLAGS_RELWITHDEBINFO        CMAKE_C_FLAGS_RELWITHDEBINFO
  )
  foreach(CompilerFlag ${CompilerFlags})
    string(REPLACE "/MT" "/MD" ${CompilerFlag} "${${CompilerFlag}}")
  endforeach()
  add_compile_definitions(_DISABLE_EXTENDED_ALIGNED_STORAGE) # to disable a msvc error
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:100000000")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(MINGW)
    set(COMPILER "MINGW Compiler")
  elseif(UNIX)
    set(COMPILER "UNIX GNU Compiler")
  else()
    message(FATAL_ERROR "Unsupported compiler.")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()

message(=============================================================)
message("-- Cmake run for ${COMPILER}")
message(=============================================================)

#========================
#  Path Setup
#========================
set(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_PROJECT_NAME}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
set(DRIVER_CMAKE_ROOT ${CMAKE_CURRENT_LIST_DIR}/cmake)

#========================
#  Boost
#========================
if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(Boost_ARCHITECTURE "-x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit
    set(Boost_ARCHITECTURE "-x32")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

find_package(Boost COMPONENTS system date_time regex REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND EXTERNAL_LIBS ${Boost_LIBRARIES})

#========================
#  PCAP
#========================
if(WIN32)
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
  find_package(PCAP REQUIRED)
  include_directories(${PCAP_INCLUDE_DIR})
  list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
else()
  list(APPEND EXTERNAL_LIBS pcap)
endif(WIN32)

#========================
#  Eigen
#========================
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
  message(=============================================================)
  message("-- Enable Transform Fcuntions")
  message(=============================================================)
endif(${ENABLE_TRANSFORM})

#========================
#  Build Demos
#========================
if(${COMPILE_DEMOS})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/demo)
endif(${COMPILE_DEMOS})

if(${COMPILE_TOOLS})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tool)
endif(${COMPILE_TOOLS})

#========================
#  Cmake
#========================  
configure_file(
  ${DRIVER_CMAKE_ROOT}/rs_driverConfig.cmake.in
  ${DRIVER_CMAKE_ROOT}/rs_driverConfig.cmake @ONLY
)

configure_file(
  ${DRIVER_CMAKE_ROOT}/rs_driverConfigVersion.cmake.in
  ${DRIVER_CMAKE_ROOT}/rs_driverConfigVersion.cmake @ONLY
)

configure_file (
  ${CMAKE_CURRENT_LIST_DIR}/src/rs_driver/macro/version.h.in
  ${CMAKE_CURRENT_LIST_DIR}/src/rs_driver/macro/version.h @ONLY
)

if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  set(rs_driver_DIR ${DRIVER_CMAKE_ROOT} PARENT_SCOPE)
endif()

#========================
#  Install & Uninstall
#========================
if(UNIX AND ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  install(FILES ${DRIVER_CMAKE_ROOT}/rs_driverConfig.cmake
                ${DRIVER_CMAKE_ROOT}/rs_driverConfigVersion.cmake
          DESTINATION ${INSTALL_CMAKE_DIR}/${CMAKE_PROJECT_NAME})

  install(DIRECTORY src/
          DESTINATION ${INSTALL_DRIVER_DIR}
          FILES_MATCHING PATTERN "*.h")

  install(DIRECTORY src/
          DESTINATION ${INSTALL_DRIVER_DIR}
          FILES_MATCHING PATTERN "*.hpp")

  if(NOT TARGET uninstall)
    configure_file(
      ${CMAKE_CURRENT_LIST_DIR}/cmake/cmake_uninstall.cmake.in
      ${PROJECT_BINARY_DIR}/cmake_uninstall.cmake @ONLY)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
  endif(NOT TARGET uninstall)
endif(UNIX AND ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})