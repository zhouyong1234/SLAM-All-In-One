cmake_minimum_required(VERSION 3.0)

project(sbgECom)

include_directories(
	${PROJECT_SOURCE_DIR}/src/
	${PROJECT_SOURCE_DIR}/common/
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/)

file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/src/*.c)
file(GLOB_RECURSE COMMON_SRC ${PROJECT_SOURCE_DIR}/common/*.c)

#
#	Exclude platform specific files
#
if (NOT MSVC)
	list(REMOVE_ITEM COMMON_SRC "${PROJECT_SOURCE_DIR}/common/interfaces/sbgInterfaceSerialWin.c")
else ()
	list(REMOVE_ITEM COMMON_SRC "${PROJECT_SOURCE_DIR}/common/interfaces/sbgInterfaceSerialUnix.c")
endif()

add_library(sbgECom STATIC ${SRC} ${COMMON_SRC})

if (MSVC)
	target_link_libraries(sbgECom Ws2_32)
endif()

