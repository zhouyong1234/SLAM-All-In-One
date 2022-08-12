####################################################################
#  This file is part of easy_config.                               #
#                                                                  #
#  Copyright (c) 2021-2022 Incloon                                 #
#                                                                  #
# easy_config is free software : you can redistribute it and/or    #
# modify it under the terms of the GNU Lesser General Public       #
# License as published by the Free Software Foundation, either     #
# version 3 of the License, or (at your option) any later version. #
#                                                                  #
# easy_config is distributed in the hope that it will be useful,   #
# but WITHOUT ANY WARRANTY; without even the implied warranty of   #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the      #
# GNU Lesser General Public License for more details.              #
#                                                                  #
# You should have received a copy of the GNU Lesser General Public #
# License along with easy_config.                                  #
# If not, see < https://www.gnu.org/licenses/>.                    #
####################################################################

cmake_minimum_required(VERSION 3.5)
project(easy_config VERSION 0.0.1 LANGUAGES CXX)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
	include(GNUInstallDirs)
endif()

add_library(interpreter INTERFACE)

target_include_directories(interpreter INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_executable(compiler
	src/compiler.cpp
)
target_link_libraries(compiler PRIVATE
	interpreter
)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	enable_testing()
	add_subdirectory(tests)
else()
	if(EZCFG_STRUCT_HEADER_FILE)
		FOREACH(file_path ${EZCFG_STRUCT_HEADER_FILE})
			if((NOT EXISTS ${file_path}) OR (IS_DIRECTORY ${file_path}))
				message(SEND_ERROR "File ${file_path} is not existence!")
			endif()
			file(RELATIVE_PATH file_path "${CMAKE_SOURCE_DIR}" "${file_path}")
			list(APPEND EZCFG_STRUCT_HEADER_FILE_RELATIVE ${file_path})
		ENDFOREACH()

		set(GEN_FILE ${CMAKE_CURRENT_BINARY_DIR}/_ezcfg_gen_code/_ezcfg_gen_file.cpp)
		file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/_ezcfg_gen_code/)
		add_custom_command(
			OUTPUT ${GEN_FILE}
			COMMAND compiler ${EZCFG_STRUCT_HEADER_FILE_RELATIVE} ${GEN_FILE}
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
			DEPENDS compiler
		)
		if(${CMAKE_VERSION} VERSION_LESS 3.12)
			add_library(interpreter_extern STATIC
				${GEN_FILE}
				)
		else()
			add_library(interpreter_extern OBJECT
				${GEN_FILE}
				)
		endif()
		add_library(ezcfg::ezcfg ALIAS interpreter_extern)
		target_link_libraries(interpreter_extern PUBLIC
			interpreter
		)
		target_include_directories(interpreter_extern PRIVATE
			${CMAKE_SOURCE_DIR}
		)
	else()
		add_library(ezcfg::ezcfg ALIAS interpreter)
		message("EZCFG_STRUCT_HEADER_FILE not set")
	endif()
endif()
