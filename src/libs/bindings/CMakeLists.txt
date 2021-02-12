find_package(Corrade REQUIRED Main)

if (BUILD_GUI_APPS)
    add_definitions(-DWITH_GUI=1)
endif()

pybind11_add_module(voxel_env voxel_env.cpp)
target_link_libraries(voxel_env PUBLIC scenarios magnum_rendering)

if (NOT CORRADE_TARGET_APPLE)
    target_link_libraries(voxel_env PUBLIC v4r_rendering)
endif ()

if (BUILD_GUI_APPS)
    target_link_libraries(voxel_env PUBLIC viewer Magnum::Application)
endif()
