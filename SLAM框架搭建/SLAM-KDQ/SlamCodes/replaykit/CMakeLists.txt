cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)

set(REPLAYKIT_ROOT ${CMAKE_CURRENT_LIST_DIR})

if (NOT DEFINED ENV{HOVER_REPO_ROOT})
  project(replaykit)
  set(CMAKE_INSTALL_PREFIX "../../env/")
  include_directories(/usr/local/include)
  link_directories(/usr/local/lib)

  find_package(Protobuf REQUIRED)
  find_package(OpenCV REQUIRED)
  if(PROTOBUF_FOUND)
  else()
      message (FATAL_ERROR "Cannot find Protobuf")
  endif()

  # Generate the .h and .cxx files
  PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/replaykit.proto proto/image.proto proto/rovio.proto)

  add_library(replay_proto ${PROTO_SRCS})
  target_include_directories(replay_proto PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(replay_proto ${PROTOBUF_LIBRARIES})

  add_library(replaykit src/Recorder.cpp)
  target_include_directories(replaykit PUBLIC include)
  target_link_libraries(replaykit replay_proto)

  add_executable(unittest unittest/main.cpp)
  target_link_libraries(unittest gtest replaykit)

  add_executable(functional_test test/main.cpp)
  target_link_libraries(functional_test replay_proto replaykit)

  add_executable(nanobag test/nanobag.cpp)
  target_link_libraries(nanobag replay_proto replaykit nanomsg)
  
  include_directories(include  ${OpenCV_INCLUDE_DIRS} )
  add_executable(svorecord test/record_svo_format.cpp)
  target_link_libraries(svorecord replay_proto replaykit nanomsg ${OpenCV_LIBS})

  INSTALL(TARGETS replaykit 
        ARCHIVE DESTINATION lib/${PROJECT_NAME})
  INSTALL(TARGETS replay_proto 
        ARCHIVE DESTINATION lib/${PROJECT_NAME})
  INSTALL(DIRECTORY include/${PROJECT_NAME} DESTINATION include) 
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/replaykit.pb.h
    ${CMAKE_CURRENT_BINARY_DIR}/rovio.pb.h 
    ${CMAKE_CURRENT_BINARY_DIR}/image.pb.h
    DESTINATION include/${PROJECT_NAME})

else()
    add_subdirectory(${REPLAYKIT_ROOT}/cmake)
endif()
