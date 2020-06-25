corrade_add_resource(TexturedTriangle_RESOURCES resources.conf)

add_executable(textured_triangle textured_triangle.cpp textured_triangle_shader.cpp ${TexturedTriangle_RESOURCES})
target_link_libraries(textured_triangle PRIVATE
        Corrade::Main
        Magnum::Application
        Magnum::GL
        Magnum::Magnum
        Magnum::Trade
        )

# So the TgaImporter gets built implicitly
add_dependencies(textured_triangle Magnum::TgaImporter)
