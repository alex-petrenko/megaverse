add_subdirectory(v4r EXCLUDE_FROM_ALL)

add_subdirectory(corrade EXCLUDE_FROM_ALL)

if (BUILD_GUI_APPS)
    set(WITH_SDL2APPLICATION ON CACHE BOOL "" FORCE)
else()
    message(STATUS "Build without GUI apps!")
endif()

set(WITH_WINDOWLESSEGLAPPLICATION ON CACHE BOOL "" FORCE)
set(WITH_TGAIMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(magnum EXCLUDE_FROM_ALL)

#find_package(Bullet REQUIRED)

set(WITH_BULLET ON CACHE BOOL "" FORCE)
add_subdirectory(magnum-integration EXCLUDE_FROM_ALL)

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED GL MeshTools Primitives SceneGraph Shaders WindowlessEglApplication Trade)
if (BUILD_GUI_APPS)
    find_package(Magnum REQUIRED Sdl2Application)
endif()
find_package(MagnumIntegration REQUIRED Bullet)

add_subdirectory(googletest-1.10.0)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_subdirectory(glad)

add_subdirectory(pybind11)
