cmake_minimum_required(VERSION 3.10)
project(librendering VERSION 0.1 LANGUAGES CXX)

add_library_default(rendering)
target_link_libraries(rendering PUBLIC env Magnum::MeshTools Magnum::Primitives)
