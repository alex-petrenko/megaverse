#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/Shaders/Phong.h>

namespace Magnum { namespace Examples {

using namespace Magnum::Math::Literals;

typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

class PickableObject: public Object3D, SceneGraph::Drawable3D {
public:
    explicit PickableObject(UnsignedInt id, Shaders::Phong& shader, const Color3& color, GL::Mesh& mesh, Object3D& parent, SceneGraph::DrawableGroup3D& drawables):
        Object3D{&parent}, SceneGraph::Drawable3D{*this, &drawables}, _id{id}, _selected{false}, _shader(shader), _color{color}, _mesh(mesh)
    {}

    void setSelected(bool selected) { _selected = selected; }

private:
    virtual void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override {
        _shader.setTransformationMatrix(transformationMatrix)
                .setNormalMatrix(transformationMatrix.normalMatrix())
                .setProjectionMatrix(camera.projectionMatrix())
                .setAmbientColor(_selected ? _color*0.3f : Color3{})
                .setDiffuseColor(_color*(_selected ? 2.0f : 1.0f))
                        /* relative to the camera */
                .setLightPosition({13.0f, 2.0f, 5.0f})
                .setObjectId(_id)
                .draw(_mesh);
    }

    UnsignedInt _id;
    bool _selected;
    Shaders::Phong& _shader;
    Color3 _color;
    GL::Mesh& _mesh;
};

class PickingExample: public Platform::Application {
public:
    explicit PickingExample(const Arguments& arguments);

private:
    void drawEvent() override;
    void mousePressEvent(MouseEvent& event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;
    void mouseReleaseEvent(MouseEvent& event) override;

    Scene3D _scene;
    Object3D* _cameraObject;
    SceneGraph::Camera3D* _camera;
    SceneGraph::DrawableGroup3D _drawables;

    Shaders::Phong _shader{Shaders::Phong::Flag::ObjectId};
    GL::Mesh _cube, _plane, _sphere;

    enum { ObjectCount = 6 };
    PickableObject* _objects[ObjectCount];

    GL::Framebuffer _framebuffer;
    GL::Renderbuffer _color, _objectId, _depth;

    Vector2i _previousMousePosition, _mousePressPosition;
};

PickingExample::PickingExample(const Arguments& arguments): Platform::Application{arguments, Configuration{}.setTitle("Magnum object picking_objects example")}, _framebuffer{GL::defaultFramebuffer.viewport()} {
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);

    /* Global renderer configuration */
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

    /* Configure framebuffer. Using a 32-bit int for object ID, which is likely
       enough. Use a smaller type if you have less objects to save memory. */
    _color.setStorage(GL::RenderbufferFormat::RGBA8, GL::defaultFramebuffer.viewport().size());
    _objectId.setStorage(GL::RenderbufferFormat::R32UI, GL::defaultFramebuffer.viewport().size());
    _depth.setStorage(GL::RenderbufferFormat::DepthComponent24, GL::defaultFramebuffer.viewport().size());
    _framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, _color)
            .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1}, _objectId)
            .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, _depth)
            .mapForDraw({{Shaders::Phong::ColorOutput, GL::Framebuffer::ColorAttachment{0}},
                         {Shaders::Phong::ObjectIdOutput, GL::Framebuffer::ColorAttachment{1}}});
    CORRADE_INTERNAL_ASSERT(_framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    /* Set up meshes */
    _cube = MeshTools::compile(Primitives::cubeSolid());
    _sphere = MeshTools::compile(Primitives::uvSphereSolid(16, 32));
    _plane = MeshTools::compile(Primitives::planeSolid());

    /* Set up objects */
    (*(_objects[0] = new PickableObject{1, _shader, 0x3bd267_rgbf, _cube, _scene, _drawables}))
            .rotate(34.0_degf, Vector3(1.0f).normalized())
            .translate({1.0f, 0.3f, -1.2f});
    (*(_objects[1] = new PickableObject{2, _shader, 0x2f83cc_rgbf, _sphere, _scene, _drawables}))
            .translate({-1.2f, -0.3f, -0.2f});
    (*(_objects[2] = new PickableObject{3, _shader, 0xdcdcdc_rgbf, _plane, _scene, _drawables}))
            .rotate(278.0_degf, Vector3(1.0f).normalized())
            .scale(Vector3(0.45f))
            .translate({-1.0f, 1.2f, 1.5f});
    (*(_objects[3] = new PickableObject{4, _shader, 0xc7cf2f_rgbf, _sphere, _scene, _drawables}))
            .translate({-0.2f, -1.7f, -2.7f});
    (*(_objects[4] = new PickableObject{5, _shader, 0xcd3431_rgbf, _sphere, _scene, _drawables}))
            .translate({0.7f, 0.6f, 2.2f})
            .scale(Vector3(0.75f));
    (*(_objects[5] = new PickableObject{6, _shader, 0xa5c9ea_rgbf, _cube, _scene, _drawables}))
            .rotate(-92.0_degf, Vector3(1.0f).normalized())
            .scale(Vector3(0.25f))
            .translate({-0.5f, -0.3f, 1.8f});

    /* Configure camera */
    _cameraObject = new Object3D{&_scene};
    _cameraObject->translate(Vector3::zAxis(8.0f));
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(Matrix4::perspectiveProjection(35.0_degf, 4.0f/3.0f, 0.001f, 100.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());
}

void PickingExample::drawEvent() {
    /* Draw to custom framebuffer */
    _framebuffer
            .clearColor(0, Color3{0.125f})
            .clearColor(1, Vector4ui{})
            .clearDepth(1.0f)
            .bind();
    _camera->draw(_drawables);

    /* Bind the main buffer back */
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth)
            .bind();

    /* Blit color to window framebuffer */
    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
    GL::AbstractFramebuffer::blit(_framebuffer, GL::defaultFramebuffer,
                                  {{}, _framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    swapBuffers();
}

void PickingExample::mousePressEvent(MouseEvent& event) {
    if(event.button() != MouseEvent::Button::Left) return;

    _previousMousePosition = _mousePressPosition = event.position();
    event.setAccepted();
}

void PickingExample::mouseMoveEvent(MouseMoveEvent& event) {
    if(!(event.buttons() & MouseMoveEvent::Button::Left)) return;

    /* We have to take window size, not framebuffer size, since the position is
       in window coordinates and the two can be different on HiDPI systems */
    const Vector2 delta = 3.0f*
                          Vector2{event.position() - _previousMousePosition}/
                          Vector2{windowSize()};

    (*_cameraObject)
            .rotate(Rad{-delta.y()}, _cameraObject->transformation().right().normalized())
            .rotateY(Rad{-delta.x()});

    _previousMousePosition = event.position();
    event.setAccepted();
    redraw();
}

void PickingExample::mouseReleaseEvent(MouseEvent& event) {
    if(event.button() != MouseEvent::Button::Left || _mousePressPosition != event.position()) return;

    /* First scale the position from being relative to window size to being
       relative to framebuffer size as those two can be different on HiDPI
       systems */
    const Vector2i position = event.position()*Vector2{framebufferSize()}/Vector2{windowSize()};
    const Vector2i fbPosition{position.x(), GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1};

    /* Read object ID at given click position */
    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{1});
    Image2D data = _framebuffer.read(
            Range2Di::fromSize(fbPosition, {1, 1}),
            {PixelFormat::R32UI});

    /* Highlight object under mouse and deselect all other */
    for(auto* o: _objects) o->setSelected(false);
    UnsignedInt id = Containers::arrayCast<UnsignedInt>(data.data())[0];
    if(id > 0 && id < ObjectCount + 1)
        _objects[id - 1]->setSelected(true);

    event.setAccepted();
    redraw();
}

}}

MAGNUM_APPLICATION_MAIN(Magnum::Examples::PickingExample)
