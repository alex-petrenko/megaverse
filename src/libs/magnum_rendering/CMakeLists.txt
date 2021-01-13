cmake_minimum_required(VERSION 3.10)
project(libgfx-magnum VERSION 0.1 LANGUAGES CXX)

add_library_default(magnum_rendering)
target_link_libraries(magnum_rendering PUBLIC env rendering glad Corrade::Main Magnum::GL Magnum::WindowlessEglApplication Magnum::Shaders Magnum::MeshTools Magnum::Primitives MagnumIntegration::Bullet)
