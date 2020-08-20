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
    target_link_libraries(viewer PRIVATE util env magnum_rendering ${MAGNUM_DEPENDENCIES} Magnum::Application Magnum::Trade)

    add_app_default(wireframe wireframe.cpp)
    target_link_libraries(wireframe PRIVATE util env ${MAGNUM_DEPENDENCIES} Magnum::Application)

    add_app_default(MyApplication MyApplication.cpp)
    target_link_libraries(MyApplication PRIVATE util ${MAGNUM_DEPENDENCIES} Magnum::Application)

    add_app_default(test_performance test_performance.cpp)
    target_link_libraries(test_performance PRIVATE util ${MAGNUM_DEPENDENCIES} Magnum::WindowlessEglApplication ${OpenCV_LIBS})
endif()

add_app_default(voxel_env_app voxel_env_app.cpp)
target_link_libraries(voxel_env_app PRIVATE util env magnum_rendering ${MAGNUM_DEPENDENCIES} Magnum::WindowlessEglApplication ${OpenCV_LIBS})


# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT viewer)
