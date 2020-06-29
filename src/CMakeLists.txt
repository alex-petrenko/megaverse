include(cmake/util.cmake)
common_settings()

add_subdirectory(3rdparty)
add_subdirectory(util)

find_package(Corrade REQUIRED Main)

find_package(Magnum REQUIRED
        GL
        MeshTools
        Primitives
        SceneGraph
        Shaders
        Sdl2Application
        WindowlessEglApplication
        Trade
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

set(MAGNUM_DEPENDENCIES
        Corrade::Main
        Magnum::GL
        Magnum::Magnum
        Magnum::MeshTools
        Magnum::Primitives
        Magnum::SceneGraph
        Magnum::Shaders
)

add_executable(MyApplication MyApplication.cpp)
target_link_libraries(MyApplication PRIVATE util ${MAGNUM_DEPENDENCIES})
target_link_libraries(MyApplication PRIVATE Magnum::Application)

add_executable(test_performance test_performance.cpp)
target_link_libraries(test_performance PRIVATE util ${MAGNUM_DEPENDENCIES} ${OpenCV_LIBS})
target_link_libraries(test_performance PRIVATE Magnum::WindowlessEglApplication)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT MyApplication)

add_subdirectory(examples)
add_subdirectory(test)