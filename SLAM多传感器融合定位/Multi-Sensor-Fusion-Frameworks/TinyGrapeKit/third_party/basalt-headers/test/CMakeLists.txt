cmake_minimum_required(VERSION 3.10)

if(NOT TARGET gtest_main)
    add_subdirectory(googletest EXCLUDE_FROM_ALL)
endif(NOT TARGET gtest_main)

include_directories(include)

add_executable(test_image src/test_image.cpp)
target_link_libraries(test_image gtest gtest_main)

add_executable(test_spline src/test_spline.cpp)
target_link_libraries(test_spline gtest gtest_main)

add_executable(test_spline_se3 src/test_spline_se3.cpp)
target_link_libraries(test_spline_se3 gtest gtest_main)

add_executable(test_camera src/test_camera.cpp)
target_link_libraries(test_camera gtest gtest_main)

add_executable(test_sophus src/test_sophus.cpp)
target_link_libraries(test_sophus gtest gtest_main)

add_executable(test_preintegration src/test_preintegration.cpp)
target_link_libraries(test_preintegration gtest gtest_main ${TBB_LIBRARIES})

add_executable(test_ceres_spline_helper src/test_ceres_spline_helper.cpp)
target_link_libraries(test_ceres_spline_helper gtest gtest_main ${TBB_LIBRARIES})

# benchmarks (currently doesnt work on macOS and with clang)
if(NOT APPLE AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "ENABLE tests")
    set(BENCHMARK_ENABLE_EXCEPTIONS OFF CACHE BOOL "ENABLE exceptions")
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "ENABLE gtests")
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "ENABLE install")
    add_subdirectory(benchmark)
    include_directories(benchmark/include)

    add_executable(benchmark_camera src/benchmark_camera.cpp)
    target_link_libraries(benchmark_camera benchmark)
endif()

include(GoogleTest)  # for gtest_discover_test

enable_testing()

gtest_discover_tests(test_image)
gtest_discover_tests(test_spline)
gtest_discover_tests(test_camera)
gtest_discover_tests(test_sophus)
gtest_discover_tests(test_preintegration)
