#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Primitives/Cube.h>
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
#include <Magnum/ImageView.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Platform/WindowlessEglApplication.h>
#include <Magnum/PixelFormat.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>


constexpr bool viz = false;
constexpr bool hires = false;
constexpr int W = hires ? 800 : 128, H = hires ? 600 : 72;

using namespace Magnum;
using namespace Magnum::Math::Literals;


typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;


struct InstanceData {
    Matrix4 transformationMatrix;
    Matrix3x3 normalMatrix;
    Color3 color;
};


class ColoredDrawable: public SceneGraph::Drawable3D
{
public:
    explicit ColoredDrawable(
            Object3D &parentObject,
            Containers::Array<InstanceData> &instanceData,
            const Color3 &color,
            const Matrix4 &primitiveTransformation,
            SceneGraph::DrawableGroup3D& drawables
    )
    : SceneGraph::Drawable3D{parentObject, &drawables}
    , _instanceData(instanceData)
    , _color{color}
    , _primitiveTransformation{primitiveTransformation}
    {}

private:
    void draw(const Matrix4& transformation, SceneGraph::Camera3D&) override {
        const Matrix4 t = transformation*_primitiveTransformation;
        arrayAppend(_instanceData, Containers::InPlaceInit, t, t.normalMatrix(), _color);
    }

    Containers::Array<InstanceData>& _instanceData;
    Color3 _color;
    Matrix4 _primitiveTransformation;
};


class WindowlessTestApp: public Platform::WindowlessApplication {
public:
    explicit WindowlessTestApp(const Arguments& arguments);

    int exec() override;

    void tick();
    void render();

private:
    Scene3D _scene;
    Object3D* _cameraObject;
    SceneGraph::Camera3D* _camera;
    SceneGraph::DrawableGroup3D _drawables;

    std::vector<std::unique_ptr<Object3D>> voxelObjects;
    std::vector<std::unique_ptr<ColoredDrawable>> instancedVoxels;

    Shaders::Phong _shader{NoCreate};

    Vector2i framebufferSize{W, H};

    GL::Framebuffer framebuffer;
    GL::Renderbuffer colorBuffer, depthBuffer;

    enum { ObjectCount = 1 };

    GL::Mesh _cube;
    GL::Buffer voxelInstanceBuffer{NoCreate};
    Containers::Array<InstanceData> voxelInstanceData;

    int direction = -1;

    Containers::Array<uint8_t> rgbaFrame;
    MutableImageView2D rgbaImageView{PixelFormat::RGBA8Unorm, framebufferSize};
};

WindowlessTestApp::WindowlessTestApp(const Arguments& arguments)
    : Platform::WindowlessApplication{arguments}
    , framebuffer{Magnum::Range2Di{{}, framebufferSize}}
{
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    colorBuffer.setStorage(GL::RenderbufferFormat::SRGB8Alpha8, framebufferSize);
    depthBuffer.setStorage(GL::RenderbufferFormat::DepthComponent24, framebufferSize);

    framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer);
    framebuffer.attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, depthBuffer);

    framebuffer.clearColor(0, Color3{0.125f}).clearDepth(1.0).bind();

    CORRADE_INTERNAL_ASSERT(framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    /* Global renderer configuration */
//    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

//    /* Configure framebuffer. Using a 32-bit int for object ID, which is likely
//       enough. Use a smaller type if you have less objects to save memory. */
//    colorBuffer.setStorage(GL::RenderbufferFormat::RGBA8, GL::defaultFramebuffer.viewport().size());
//    _objectId.setStorage(GL::RenderbufferFormat::R32UI, GL::defaultFramebuffer.viewport().size());
//    _depth.setStorage(GL::RenderbufferFormat::DepthComponent24, GL::defaultFramebuffer.viewport().size());
//    framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer)
//            .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1}, _objectId)
//            .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, _depth)
//            .mapForDraw({{Shaders::Phong::ColorOutput, GL::Framebuffer::ColorAttachment{0}},
//                         {Shaders::Phong::ObjectIdOutput, GL::Framebuffer::ColorAttachment{1}}});
//    CORRADE_INTERNAL_ASSERT(framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    _shader = Shaders::Phong{Shaders::Phong::Flag::VertexColor|Shaders::Phong::Flag::InstancedTransformation};
    _shader.setAmbientColor(0x111111_rgbf).setSpecularColor(0x330000_rgbf).setLightPosition({10.0f, 15.0f, 5.0f});

    /* Set up meshes */
    _cube = MeshTools::compile(Primitives::cubeSolid());

    voxelInstanceBuffer = GL::Buffer{};
    _cube.addVertexBufferInstanced(
            voxelInstanceBuffer, 1, 0,
            Shaders::Phong::TransformationMatrix{},
            Shaders::Phong::NormalMatrix{},
            Shaders::Phong::Color3{}
    );



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
//            auto transformation = Matrix4::scaling(Vector3{0.45f}) * Matrix4::translation({float(-x), 0, float(-z)});
            auto transformation = Matrix4::scaling(Vector3{1.0f});

            auto voxelObject = std::make_unique<Object3D>(&_scene);
            voxelObject->scale(Vector3(0.20f)).translate({float(-x), 0, float(-z)});

            auto voxel = std::make_unique<ColoredDrawable>(*voxelObject, voxelInstanceData, 0xa5c9ea_rgbf, transformation, _drawables);
//            voxel->scale(Vector3(0.20f)).translate({float(-x), 0, float(-z)});

            voxelObjects.push_back(std::move(voxelObject));
            instancedVoxels.push_back(std::move(voxel));

//            std::unique_ptr<VoxelCube> voxel{new VoxelCube{uint(100 * x + z), _shader, 0xa5c9ea_rgbf, cubeMesh, _scene, drawables}};

//            voxels.push_back(std::move(voxel));
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

    rgbaFrame = Containers::Array<uint8_t>{size_t(framebufferSize.x() * framebufferSize.y() * 4)};
    rgbaImageView = MutableImageView2D{PixelFormat::RGBA8Unorm, framebufferSize, rgbaFrame};

    TLOG(DEBUG);
}


//void PrimitivesExample::mouseScrollEvent(MouseScrollEvent& event) {
//    if(!event.offset().y()) return;
//
//    /* Distance to origin */
//    const Float distance = _cameraObject->transformation().translation().z();
//
//    /* Move 15% of the distance back or forward */
//    _cameraObject->translate(Vector3::zAxis(distance*(1.0f - (event.offset().y() > 0 ? 1/0.85f : 0.85f))));
//
//    redraw();
//}


void WindowlessTestApp::render()
{
//    TLOG(DEBUG) << "draw";

//    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);

//    for (const auto &t : _translations) {
//        auto transformation = t * rotation;
//
//        _shader.setLightPosition({7.0f, 5.0f, 2.5f})
//                .setLightColor(Color3{1.0f})
//                .setDiffuseColor(_color)
//                .setAmbientColor(Color3::fromHsv({_color.hue(), 1.0f, 0.3f}))
//                .setTransformationMatrix(transformation)
//                .setNormalMatrix(transformation.normalMatrix())
//                .setProjectionMatrix(_projection)
//                .draw(_mesh);
//    }


    /* Draw to custom framebuffer */
//    _framebuffer
//            .clearColor(0, Color3{0.125f})
//            .clearColor(1, Vector4ui{})
//            .clearDepth(1.0f)
//            .bind();



    /* Draw to custom framebuffer */
    framebuffer.clearColor(0, Color3{0.125f}).clearDepth(1.0).bind();
//    _camera->draw(drawables);

    arrayResize(voxelInstanceData, 0);
    _camera->draw(_drawables);

    _shader.setProjectionMatrix(_camera->projectionMatrix());

    /* Upload instance data to the GPU (orphaning the previous buffer
       contents) and draw all cubes in one call, and all spheres (if any)
       in another call */
    voxelInstanceBuffer.setData(voxelInstanceData, GL::BufferUsage::DynamicDraw);
    _cube.setInstanceCount(voxelInstanceData.size());
    _shader.draw(_cube);


//    Image2D image{PixelFormat::RGBA8Unorm, {512, 256}, std::move(data)};


    framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0}).read(framebuffer.viewport(), rgbaImageView);

    if constexpr (viz) {
        cv::Mat mat(framebufferSize.y(), framebufferSize.x(), CV_8UC4, rgbaImageView.data());
        cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
        cv::flip(mat, mat, 0);
        cv::imshow("window", mat);
        cv::waitKey(10);
    }

//    /* Bind the main buffer back */
//    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth).bind();
//
//    /* Blit color to window framebuffer */
//    framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
//    GL::AbstractFramebuffer::blit(framebuffer, GL::defaultFramebuffer,
//                                  {{}, framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

//    swapBuffers();
}

void WindowlessTestApp::tick()
{
    for (auto &v : voxelObjects) {
        auto transformation = Matrix4::rotationY(2.0_degf);
        v->transformLocal(transformation);
    }

    _cameraObject->translate(Vector3{direction * 0.02f, 0, 0});
//    TLOG(DEBUG) << "tick " << _cameraObject->transformation().translation().x();

    const auto xCoord = _cameraObject->transformation().translation().x();
    if (fabs(double(xCoord)) > 10)
        direction *= -1;
}


int WindowlessTestApp::exec()
{
    auto eglContext = glContext();

    TLOG(DEBUG) << "OpenGL version:" << GL::Context::current().versionString();
    TLOG(DEBUG) << "OpenGL renderer:" << GL::Context::current().rendererString();

    constexpr auto nFrames = 10000;
    tprof().startTimer("loop");
    for (int i = 0; i < nFrames; ++i) {
        tick();
        render();
    }
    const auto usecPassed = tprof().stopTimer("loop");

    TLOG(DEBUG) << "FPS: " << nFrames / (usecPassed / 1e6) << " for " << nFrames << " frames";

    return EXIT_SUCCESS;
}

/* main() function implementation */
MAGNUM_WINDOWLESSAPPLICATION_MAIN(WindowlessTestApp)
