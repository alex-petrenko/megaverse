#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/CompressIndices.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Crosshair.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/Context.h>

#include <util/tiny_logger.hpp>
#include <env/env.hpp>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/Shaders.h>


using namespace Magnum;
using namespace Magnum::Math::Literals;


typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;


class FlatDrawable : public SceneGraph::Drawable3D
{
public:
    explicit FlatDrawable(Object3D &parentObject, SceneGraph::DrawableGroup3D &drawables,
                          Shaders::Flat3D & shader, GL::Mesh& mesh)
        : SceneGraph::Drawable3D{parentObject, &drawables}
        , _shader(shader), _mesh(mesh)
    {}

    void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) override
    {
        _shader
        .setTransformationProjectionMatrix(camera.projectionMatrix()*transformation)
        .draw(_mesh);
    }

private:
    Shaders::Flat3D & _shader;
    GL::Mesh& _mesh;
};



class Wireframe: public Platform::Application
{
public:
    explicit Wireframe(const Arguments& arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

private:
    std::unique_ptr<FlatDrawable> crosshairDrawable;

    Scene3D _scene;
    Object3D* cameraObject;
    SceneGraph::Camera3D* camera;
    SceneGraph::DrawableGroup3D drawables;

    Shaders::Flat3D wireframeShader{NoCreate};

    GL::Framebuffer framebuffer;
    GL::Renderbuffer colorBuffer, depthBuffer;

    GL::Mesh crosshair;

    int direction = -1;
};

Wireframe::Wireframe(const Arguments& arguments):
    Platform::Application{arguments, Configuration{}.setTitle("Magnum test")}, framebuffer{GL::defaultFramebuffer.viewport()}
{
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    colorBuffer.setStorage(GL::RenderbufferFormat::SRGB8Alpha8, GL::defaultFramebuffer.viewport().size());
    depthBuffer.setStorage(GL::RenderbufferFormat::DepthComponent24, GL::defaultFramebuffer.viewport().size());

    framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer);
    framebuffer.attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, depthBuffer);
    framebuffer.mapForDraw({
                               {Shaders::Flat3D::ColorOutput, GL::Framebuffer::ColorAttachment{0}},
                           });

    CORRADE_INTERNAL_ASSERT(framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    wireframeShader = Shaders::Flat3D{};
    wireframeShader.setColor(0xffffff_rgbf);

    crosshair = MeshTools::compile(Primitives::crosshair3D());
    crosshairDrawable = std::make_unique<FlatDrawable>(_scene, drawables, wireframeShader, crosshair);

    /* Configure camera */
    cameraObject = new Object3D{&_scene};
    cameraObject->translate(Vector3{3, 1.5, 5});
    camera = new SceneGraph::Camera3D{*cameraObject};
    camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix4::perspectiveProjection(60.0_degf, 4.0f/3.0f, 0.001f, 100.0f))
        .setViewport(GL::defaultFramebuffer.viewport().size());
}

void Wireframe::drawEvent() {
    framebuffer
        .clearColor(0, Color3{0.125f})
        .clearColor(1, Vector4ui{})
        .clearDepth(1.0f)
        .bind();

    camera->draw(drawables);

    /* Bind the main buffer back */
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth).bind();

    /* Blit color to window framebuffer */
    framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
    GL::AbstractFramebuffer::blit(framebuffer, GL::defaultFramebuffer,
                                  {{}, framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    swapBuffers();
}

/**
 * Move camera back and forth.
 */
void Wireframe::tickEvent()
{
    cameraObject->translate(Vector3{float(direction) * 0.001f, 0, 0});

    if (abs(cameraObject->transformation().translation().x()) > 18)
        direction *= -1;

    redraw();
}

MAGNUM_APPLICATION_MAIN(Wireframe)