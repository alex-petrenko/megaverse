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
    add_app_default(viewer viewer.cpp)
    target_link_libraries(viewer PRIVATE scenarios magnum_rendering ${MAGNUM_DEPENDENCIES} Magnum::Application Magnum::Trade)
endif()

add_app_default(voxel_env_app voxel_env_app.cpp)
target_link_libraries(voxel_env_app PRIVATE scenarios magnum_rendering v4r_rendering ${MAGNUM_DEPENDENCIES} Magnum::WindowlessEglApplication ${OpenCV_LIBS})

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT viewer)


add_app_default(mazegen mazegen.cpp)
target_link_libraries(mazegen PRIVATE mazes)
