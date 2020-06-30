cmake_minimum_required(VERSION 3.13)
project(sapphire-rl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

add_subdirectory(corrade EXCLUDE_FROM_ALL)

set(WITH_SDL2APPLICATION ON CACHE BOOL "" FORCE)
set(WITH_WINDOWLESSEGLAPPLICATION ON CACHE BOOL "" FORCE)
set(WITH_TGAIMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(magnum EXCLUDE_FROM_ALL)

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED GL MeshTools Primitives SceneGraph Shaders Sdl2Application WindowlessEglApplication Trade)

add_subdirectory(src)
