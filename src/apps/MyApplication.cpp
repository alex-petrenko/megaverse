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
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/Context.h>

#include <util/magnum.hpp>
#include <util/tiny_logger.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


class VoxelCube : public Object3D, SceneGraph::Drawable3D {
public:
    explicit VoxelCube(UnsignedInt id, Shaders::Phong& shader, const Color3& color, GL::Mesh& mesh, Object3D& parent, SceneGraph::DrawableGroup3D& drawables):
            Object3D{&parent}, SceneGraph::Drawable3D{*this, &drawables}, _id{id}, _shader(shader), _color{color}, _mesh(mesh)
    {}

private:
    virtual void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override {
        _shader.setTransformationMatrix(transformationMatrix)
                .setNormalMatrix(transformationMatrix.normalMatrix())
                .setProjectionMatrix(camera.projectionMatrix())
                .setDiffuseColor(_color*1.0f)
                        /* relative to the camera */
                .setLightPosition({0.0f, 5.0f, 5.0f})
                .setObjectId(_id)
                .draw(_mesh);
    }

    UnsignedInt _id;
    Shaders::Phong& _shader;
    Color3 _color;
    GL::Mesh& _mesh;
};


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


class PrimitivesExample: public Platform::Application {
public:
    explicit PrimitivesExample(const Arguments& arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

    void mouseScrollEvent(MouseScrollEvent& event) override;


    Scene3D _scene;
    Object3D* _cameraObject;
    SceneGraph::Camera3D* _camera;
    SceneGraph::DrawableGroup3D _drawables;

    std::vector<std::unique_ptr<VoxelCube>> voxels;

    Shaders::Phong _shader{Shaders::Phong::Flag::ObjectId};

    GL::Framebuffer _framebuffer;
    GL::Renderbuffer _color, _objectId, _depth;

    enum { ObjectCount = 1 };
    PickableObject* _objects[ObjectCount];

    GL::Mesh _cube;

    int direction = -1;
};

PrimitivesExample::PrimitivesExample(const Arguments& arguments):
    Platform::Application{arguments, Configuration{}.setTitle("Magnum test")}, _framebuffer{GL::defaultFramebuffer.viewport()}
{
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

//    _mesh.setPrimitive(cube.primitive())
//            .setCount(cube.indexCount())
//            .addVertexBuffer(std::move(vertices), 0, Shaders::Phong::Position{}, Shaders::Phong::Normal{})
//            .setIndexBuffer(std::move(indices), 0, compressed.second);
//
//    rotation = Matrix4{};
//
//    for (int i = 0; i < 10; ++i) {
//        auto translation = Matrix4::translation(Vector3::xAxis(i));
//        _translations.push_back(translation);
//    }

    for (int x = 0; x < 30; ++x) {
        for (int z = 0; z < 30; ++z) {
            std::unique_ptr<VoxelCube> voxel{new VoxelCube{uint(100 * x + z), _shader, 0xa5c9ea_rgbf, _cube, _scene, _drawables}};
            voxel->scale(Vector3(0.45f)).translate({float(-x), 0, float(-z)});
            voxels.push_back(std::move(voxel));
        }
    }

    /* Configure camera */
    _cameraObject = new Object3D{&_scene};
    _cameraObject->rotateX(-10.0_degf);
    _cameraObject->rotateY(20.0_degf);
    _cameraObject->translate(Vector3{-1, 4, 12});
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(Matrix4::perspectiveProjection(60.0_degf, 4.0f/3.0f, 0.001f, 100.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());
}


void PrimitivesExample::mouseScrollEvent(MouseScrollEvent& event) {
    if(!event.offset().y()) return;

    /* Distance to origin */
    const Float distance = _cameraObject->transformation().translation().z();

    /* Move 15% of the distance back or forward */
    _cameraObject->translate(Vector3::zAxis(distance*(1.0f - (event.offset().y() > 0 ? 1/0.85f : 0.85f))));

    redraw();
}


void PrimitivesExample::drawEvent() {
//    TLOG(DEBUG) << "draw";

//    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);

//    for (const auto &t : _translations) {
//        auto transformation = t * rotation;
//
//        _shader.setLightPosition({7.0f, 5.0f, 2.5f})
//                .setLightColor(Color3{1.0f})
//                .setDiffuseColor(colorBuffer)
//                .setAmbientColor(Color3::fromHsv({colorBuffer.hue(), 1.0f, 0.3f}))
//                .setTransformationMatrix(transformation)
//                .setNormalMatrix(transformation.normalMatrix())
//                .setProjectionMatrix(_projection)
//                .draw(_mesh);
//    }


    /* Draw to custom framebuffer */
//    framebuffer
//            .clearColor(0, Color3{0.125f})
//            .clearColor(1, Vector4ui{})
//            .clearDepth(1.0f)
//            .bind();



    /* Draw to custom framebuffer */
    _framebuffer
            .clearColor(0, Color3{0.125f})
            .clearColor(1, Vector4ui{})
            .clearDepth(1.0f)
            .bind();
    _camera->draw(_drawables);

    /* Bind the main buffer back */
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth).bind();

    /* Blit color to window framebuffer */
    _framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
    GL::AbstractFramebuffer::blit(_framebuffer, GL::defaultFramebuffer,
                                  {{}, _framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    swapBuffers();
}

void PrimitivesExample::tickEvent() {
    _cameraObject->translate(Vector3{direction * 0.2f, 0, 0});
//    TLOG(DEBUG) << "tick" << _cameraObject->transformation().translation().x();

    if (abs(_cameraObject->transformation().translation().x()) > 12)
        direction *= -1;

    redraw();
}

MAGNUM_APPLICATION_MAIN(PrimitivesExample)