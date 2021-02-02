cmake_minimum_required(VERSION 3.10)
project(libgfx-magnum VERSION 0.1 LANGUAGES CXX)

add_library_default(magnum_rendering)
target_link_libraries(magnum_rendering PUBLIC env rendering Corrade::Main Magnum::GL Magnum::Shaders Magnum::MeshTools Magnum::Primitives MagnumIntegration::Bullet)

find_package (Corrade REQUIRED Main)
if (NOT CORRADE_TARGET_APPLE)
    find_package(Magnum REQUIRED WindowlessEglApplication)
    target_link_libraries(magnum_rendering PUBLIC glad Magnum::WindowlessEglApplication)
else ()
    find_package(Magnum REQUIRED WindowlessCglApplication)
    target_link_libraries(magnum_rendering PUBLIC Magnum::WindowlessCglApplication)
endif ()
