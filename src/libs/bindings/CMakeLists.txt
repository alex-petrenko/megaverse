find_package(Corrade REQUIRED Main)

pybind11_add_module(voxel_env voxel_env.cpp)
target_link_libraries(voxel_env PUBLIC scenarios magnum_rendering)

if (NOT CORRADE_TARGET_APPLE)
    target_link_libraries(voxel_env PUBLIC v4r_rendering)
endif ()
