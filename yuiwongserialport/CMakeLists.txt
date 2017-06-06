##
# This library is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
##
# cmake and project
#
cmake_minimum_required(VERSION 2.8.3)
project(yuiwongserialport)
##
# basic config
#
# build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# the path to the top level of the build tree
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
# source root
set(yuiwongserialport_SOURCE_DIR ${PROJECT_SOURCE_DIR})
message("** PROJECT_NAME " ${PROJECT_NAME})
message(STATUS "CMAKE_BUILD_TYPE " ${CMAKE_BUILD_TYPE})
message(STATUS "PROJECT_SOURCE_DIR " ${PROJECT_SOURCE_DIR})
message(STATUS "CMAKE_BINARY_DIR " ${CMAKE_BINARY_DIR})
message(STATUS "yuiwongserialport_SOURCE_DIR "
  ${yuiwongserialport_SOURCE_DIR})
##
# basic CXX_FLAGS
#
# check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "using flag -std=c++0x.")
else()
  message(FATAL_ERROR
    "the compiler ${CMAKE_CXX_COMPILER} has no C++11 support. "
    "please use a different C++ compiler.")
endif()
# other basic: warnings ..
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wundef")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunreachable-code")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winline")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
message(STATUS "CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS})
include_directories(include)
#include_directories(${CMAKE_SOURCE_DIR})
##
add_library(${PROJECT_NAME} SHARED
  serialport.cpp)
add_library(${PROJECT_NAME}_static
  serialport.cpp)
# 指定静态库的输出名称
set_target_properties(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME
  ${PROJECT_NAME})
# 使动态库和静态库同时存在
set_target_properties(${PROJECT_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECT_NAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
# 指定动态库版本
# VERSION 动态库版本
# SOVERSION API 版本
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0 SOVERSION 1)
##
# test
#
add_subdirectory(${PROJECT_SOURCE_DIR}/test)
## gen package config
set(yuiwongserialport_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(yuiwongserialport_LIBRARIES
  ${CMAKE_INSTALL_PREFIX}/lib/lib${PROJECT_NAME}.so)
configure_file(yuiwongserialport-config.cmake.in
  ${PROJECT_BINARY_DIR}/yuiwongserialport-config.cmake @ONLY)
export(PACKAGE yuiwongserialport)
##
# install
#
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_static
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
set(ConfigPackageLocation lib)
install(FILES ${PROJECT_BINARY_DIR}/yuiwongserialport-config.cmake
  DESTINATION ${ConfigPackageLocation})
