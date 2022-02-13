find_package(Corrade REQUIRED Main)
set(MAGNUM_DEPENDENCIES
        Corrade::Main
        Magnum::GL
        Magnum::Magnum
        Magnum::MeshTools
        Magnum::Primitives
        Magnum::SceneGraph
        Magnum::Shaders
)

if (BUILD_GUI_APPS)
    set(VIEWER_APP_SOURCES viewer_app.cpp viewer_args.cpp)
    add_app_default(viewer_app "${VIEWER_APP_SOURCES}")  # list of sources gotta be in quites
    target_link_libraries(viewer_app PRIVATE scenarios viewer Magnum::Application)
endif()

find_package(ZLIB)

set(MEGAVERSE_TEST_APP_SOURCES megaverse_test_app.cpp viewer_args.cpp)
add_app_default(megaverse_test_app "${MEGAVERSE_TEST_APP_SOURCES}")
target_link_libraries(megaverse_test_app PRIVATE scenarios magnum_rendering ${MAGNUM_DEPENDENCIES} ${OpenCV_LIBS})

if (ZLIB_FOUND)
    target_link_libraries(megaverse_test_app PRIVATE ${ZLIB_LIBRARIES})
endif ()

if (NOT CORRADE_TARGET_APPLE)
    target_link_libraries(megaverse_test_app PRIVATE v4r_rendering)
endif ()

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT viewer)


add_app_default(mazegen mazegen.cpp)
target_link_libraries(mazegen PRIVATE mazes)
