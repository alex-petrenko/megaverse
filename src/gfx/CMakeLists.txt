cmake_minimum_required(VERSION 3.13)
project(libgfx VERSION 0.1 LANGUAGES CXX)

add_library_default(gfx)
target_link_libraries(gfx util glad Corrade::Main Magnum::GL Magnum::WindowlessEglApplication)
