find_package(Corrade REQUIRED Main)

if (BUILD_GUI_APPS)
    add_definitions(-DWITH_GUI=1)
endif()

pybind11_add_module(megaverse megaverse.cpp)
target_link_libraries(megaverse PUBLIC scenarios magnum_rendering)

if (NOT CORRADE_TARGET_APPLE)
    target_link_libraries(megaverse PUBLIC v4r_rendering)
endif ()

if (BUILD_GUI_APPS)
    target_link_libraries(megaverse PUBLIC viewer Magnum::Application)
endif()
