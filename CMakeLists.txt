cmake_minimum_required(VERSION 3.16)
project(ws30_lidar_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

add_library(ws30_lidar_core
  src/client.cpp
  src/frame_assembler.cpp
  src/packet_parser.cpp
  src/capture_log.cpp
  src/pcd_writer.cpp
  src/udp.cpp)

target_include_directories(ws30_lidar_core PUBLIC include)

target_link_libraries(ws30_lidar_core PUBLIC rt pthread)

enable_testing()

file(GLOB_RECURSE TEST_SOURCES tests/*_test.cpp)
foreach(_src ${TEST_SOURCES})
  get_filename_component(_name ${_src} NAME_WLE)
  add_executable(${_name} ${_src})
  target_include_directories(${_name} PRIVATE include tests)
  target_link_libraries(${_name} PRIVATE ws30_lidar_core)
  add_test(NAME ${_name} COMMAND ${_name})
endforeach()
