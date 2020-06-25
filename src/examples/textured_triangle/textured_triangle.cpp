#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/ImageView.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>

#include "textured_triangle_shader.hpp"


using namespace Magnum;


class TexturedTriangleExample: public Platform::Application {
public:
    explicit TexturedTriangleExample(const Arguments& arguments);

private:
    void drawEvent() override;

    GL::Mesh _mesh;
    TexturedTriangleShader _shader;
    GL::Texture2D _texture;
};

TexturedTriangleExample::TexturedTriangleExample(const Arguments& arguments):
        Platform::Application{arguments, Configuration{}
                .setTitle("Magnum Textured Triangle Example")}
{
    struct TriangleVertex {
        Vector2 position;
        Vector2 textureCoordinates;
    };
    const TriangleVertex data[]{
            {{-0.5f, -0.5f}, {0.0f, 0.0f}}, /* Left position and texture coordinate */
            {{ 0.5f, -0.5f}, {1.0f, 0.0f}}, /* Right position and texture coordinate */
            {{ 0.0f,  0.5f}, {0.5f, 1.0f}}  /* Top position and texture coordinate */
    };

    GL::Buffer buffer;
    buffer.setData(data);
    _mesh.setCount(3)
            .addVertexBuffer(std::move(buffer), 0,
                             TexturedTriangleShader::Position{},
                             TexturedTriangleShader::TextureCoordinates{});

    PluginManager::Manager<Trade::AbstractImporter> manager;
    Containers::Pointer<Trade::AbstractImporter> importer =
            manager.loadAndInstantiate("TgaImporter");
    if(!importer) std::exit(1);

    const Utility::Resource rs{"textured-triangle-data"};
    if(!importer->openData(rs.getRaw("stone.tga")))
        std::exit(2);

    Containers::Optional<Trade::ImageData2D> image = importer->image2D(0);
    CORRADE_INTERNAL_ASSERT(image);
    _texture.setWrapping(GL::SamplerWrapping::ClampToEdge)
            .setMagnificationFilter(GL::SamplerFilter::Linear)
            .setMinificationFilter(GL::SamplerFilter::Linear)
            .setStorage(1, GL::textureFormat(image->format()), image->size())
            .setSubImage(0, {}, *image);
}

void TexturedTriangleExample::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);

    using namespace Math::Literals;

    _shader
            .setColor(0xffb2b2_rgbf)
            .bindTexture(_texture)
            .draw(_mesh);

    swapBuffers();
}

MAGNUM_APPLICATION_MAIN(TexturedTriangleExample)