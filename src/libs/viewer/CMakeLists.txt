cmake_minimum_required(VERSION 3.10)
project(libviewer VERSION 0.1 LANGUAGES CXX)

if (BUILD_GUI_APPS)
    find_package(Corrade REQUIRED Main)
    set(MAGNUM_DEPENDENCIES
            Corrade::Main
            Magnum::GL
            Magnum::Magnum
            Magnum::MeshTools
            Magnum::Primitives
            Magnum::SceneGraph
            Magnum::Shaders)

    add_library_default(viewer)
    target_link_libraries(viewer PUBLIC env magnum_rendering ${MAGNUM_DEPENDENCIES} Magnum::Trade Magnum::Application ${OpenCV_LIBS})

    if (NOT CORRADE_TARGET_APPLE)
        target_link_libraries(viewer PUBLIC v4r_rendering)
    endif ()
endif()