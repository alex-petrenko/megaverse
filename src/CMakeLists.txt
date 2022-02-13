cmake_minimum_required(VERSION 3.13)
project(megaverse)

add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

option(BUILD_GUI_APPS "Whether to build apps that require GUI (things like SDL2)" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

include(cmake/util.cmake)
common_settings()

add_subdirectory(3rdparty)

add_compile_options(-Wall -Wextra -Wdelete-non-virtual-dtor -pedantic -Werror)
add_subdirectory(libs)

add_subdirectory(apps)
add_subdirectory(examples)
add_subdirectory(test)
