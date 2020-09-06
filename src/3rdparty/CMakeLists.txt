add_subdirectory(v4r EXCLUDE_FROM_ALL)

add_subdirectory(corrade EXCLUDE_FROM_ALL)

set(WITH_SDL2APPLICATION ON CACHE BOOL "" FORCE)
set(WITH_WINDOWLESSEGLAPPLICATION ON CACHE BOOL "" FORCE)
set(WITH_TGAIMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(magnum EXCLUDE_FROM_ALL)

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED GL MeshTools Primitives SceneGraph Shaders Sdl2Application WindowlessEglApplication Trade)

add_subdirectory(googletest-1.10.0)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_subdirectory(glad)

add_subdirectory(pybind11)
