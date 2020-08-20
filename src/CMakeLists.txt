cmake_minimum_required(VERSION 3.10)
project(sapphire-rl)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

include(cmake/util.cmake)
common_settings()

add_subdirectory(3rdparty)
add_subdirectory(libs)
add_subdirectory(apps)
add_subdirectory(examples)
add_subdirectory(test)
