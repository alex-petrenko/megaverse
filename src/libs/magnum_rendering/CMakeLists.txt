cmake_minimum_required(VERSION 3.13)
project(libgfx VERSION 0.1 LANGUAGES CXX)

add_library_default(magnum_rendering)
target_link_libraries(magnum_rendering util env glad Corrade::Main Magnum::GL Magnum::WindowlessEglApplication Magnum::Shaders Magnum::MeshTools Magnum::Primitives)
