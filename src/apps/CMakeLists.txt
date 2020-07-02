set(MAGNUM_DEPENDENCIES
        Corrade::Main
        Magnum::GL
        Magnum::Magnum
        Magnum::MeshTools
        Magnum::Primitives
        Magnum::SceneGraph
        Magnum::Shaders
)

add_app_default(MyApplication MyApplication.cpp)
target_link_libraries(MyApplication PRIVATE util ${MAGNUM_DEPENDENCIES} Magnum::Application)

add_app_default(test_performance test_performance.cpp)
target_link_libraries(test_performance PRIVATE util ${MAGNUM_DEPENDENCIES} Magnum::WindowlessEglApplication ${OpenCV_LIBS})

add_app_default(viewer viewer.cpp)
target_link_libraries(viewer PRIVATE util env ${MAGNUM_DEPENDENCIES} Magnum::Application)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT viewer)
