cmake_minimum_required(VERSION 3.13)
project(libgfx-v4r VERSION 0.1 LANGUAGES CXX)

add_library_default(v4r_rendering)
target_link_libraries(v4r_rendering util env rendering v4r_headless v4r_debug Magnum::Primitives)
