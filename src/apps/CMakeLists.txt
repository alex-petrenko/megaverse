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
    add_app_default(viewer_app viewer_app.cpp)
    target_link_libraries(viewer_app PRIVATE scenarios viewer Magnum::Application)
endif()

add_app_default(voxel_env_app voxel_env_app.cpp)
target_link_libraries(voxel_env_app PRIVATE scenarios magnum_rendering ${MAGNUM_DEPENDENCIES} ${OpenCV_LIBS}) 

if (NOT CORRADE_TARGET_APPLE)
    target_link_libraries(voxel_env_app PRIVATE v4r_rendering)
endif ()

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT viewer)


add_app_default(mazegen mazegen.cpp)
target_link_libraries(mazegen PRIVATE mazes)
