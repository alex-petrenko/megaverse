add_subdirectory(corrade EXCLUDE_FROM_ALL)
find_package(Corrade REQUIRED Main)

if (NOT CORRADE_TARGET_APPLE)
    add_subdirectory(v4r EXCLUDE_FROM_ALL)
endif ()


if (BUILD_GUI_APPS)
    set(WITH_SDL2APPLICATION ON CACHE BOOL "" FORCE)
else()
    message(STATUS "Build without GUI apps!")
endif()

if (NOT CORRADE_TARGET_APPLE)
    set(WITH_WINDOWLESSEGLAPPLICATION ON CACHE BOOL "" FORCE)
else ()
    set(WITH_WINDOWLESSCGLAPPLICATION ON CACHE BOOL "WITH_WINDOWLESSCGLAPPLICATION" FORCE)
endif ()
set(WITH_TGAIMPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(magnum EXCLUDE_FROM_ALL)

#find_package(Bullet REQUIRED)

set(WITH_BULLET ON CACHE BOOL "" FORCE)
add_subdirectory(magnum-integration EXCLUDE_FROM_ALL)

find_package(Magnum REQUIRED GL MeshTools Primitives SceneGraph Shaders Trade)
if (BUILD_GUI_APPS)
    find_package(Magnum REQUIRED Sdl2Application)
endif()
find_package(MagnumIntegration REQUIRED Bullet)

add_subdirectory(googletest-1.10.0)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

if (NOT CORRADE_TARGET_APPLE)
    add_subdirectory(glad)
endif ()

add_subdirectory(pybind11)
