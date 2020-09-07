pybind11_add_module(voxel_env voxel_env.cpp)
target_link_libraries(voxel_env PUBLIC util env magnum_rendering v4r_rendering ${OpenCV_LIBS})
