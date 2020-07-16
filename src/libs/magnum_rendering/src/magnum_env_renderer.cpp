#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Platform/WindowlessEglApplication.h>
#include <Magnum/PixelFormat.h>

#include <magnum_rendering/magnum_env_renderer.hpp>
#include <util/tiny_logger.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


struct InstanceData {
    Magnum::Matrix4 transformationMatrix;
    Magnum::Matrix3x3 normalMatrix;
    Magnum::Color3 color;
};


class CustomDrawable : public SceneGraph::Drawable3D
{
public:
    explicit CustomDrawable(
        Object3D &parentObject,
        Containers::Array<InstanceData> &instanceData,
        const Color3 &color,
        const Matrix4 &primitiveTransformation,
        SceneGraph::DrawableGroup3D &drawables
    )
        : SceneGraph::Drawable3D{parentObject, &drawables}, _instanceData(instanceData), _color{color},
          _primitiveTransformation{primitiveTransformation}
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


class SimpleDrawable3D : public Object3D, public SceneGraph::Drawable3D
{
public:
    explicit SimpleDrawable3D(Object3D &parentObject, SceneGraph::DrawableGroup3D &drawables,
                              Shaders::Phong & shader, GL::Mesh& mesh, const Color3 &color)
        : Object3D{&parentObject}
        , SceneGraph::Drawable3D{*this, &drawables}
        , _shader(shader), _mesh(mesh), _color(color)
    {}

    void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override
    {
        _shader.setTransformationMatrix(transformationMatrix)
            .setNormalMatrix(transformationMatrix.normalMatrix())
            .setProjectionMatrix(camera.projectionMatrix())
            .setAmbientColor(0.4 * _color)
            .setDiffuseColor(_color)
            .setShininess(150)
                /* relative to the camera */
            .setLightPosition({13.0f, 2.0f, 5.0f})
            .setLightColor(0xaaaaaa_rgbf)
            .draw(_mesh);
    }

private:
    Shaders::Phong& _shader;
    GL::Mesh& _mesh;
    Color3 _color;
};


struct MagnumEnvRenderer::Impl : public Platform::WindowlessApplication
{
public:
    explicit Impl(const Arguments& arguments, Env &env, int w, int h);

    // this is pure virtual in the base class, but we're not using it
    virtual int exec() override { return 0; }

    void draw(Env &env);

    const uint8_t * getObservation(int agentIdx) const;

public:
    Vector2i framebufferSize;

    std::vector<std::unique_ptr<SimpleDrawable3D>> agentDrawables;

    std::vector<std::unique_ptr<Object3D>> layoutObjects;
    std::vector<std::unique_ptr<CustomDrawable>> instancedLayoutDrawables;

    std::unique_ptr<SimpleDrawable3D> exitPadDrawable;

    GL::Buffer voxelInstanceBuffer{NoCreate};
    Containers::Array<InstanceData> voxelInstanceData;

    SceneGraph::DrawableGroup3D drawables;

    Shaders::Phong shader{NoCreate};
    Shaders::Phong shaderInstanced{NoCreate};

    GL::Framebuffer framebuffer;
    GL::Renderbuffer colorBuffer, depthBuffer;

    GL::Mesh cubeMesh, exitPadMesh, agentMesh, agentEyeMesh;

    std::vector<Containers::Array<uint8_t>> agentFrames;
    std::vector<std::unique_ptr<MutableImageView2D>> agentImageViews;
};


MagnumEnvRenderer::Impl::Impl(const Platform::WindowlessEglApplication::Arguments &arguments, Env &env, int w, int h)
    : Platform::WindowlessApplication{arguments}
    , framebufferSize{w, h}
    , framebuffer{Magnum::Range2Di{{}, framebufferSize}}
{
    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    colorBuffer.setStorage(GL::RenderbufferFormat::SRGB8Alpha8, framebufferSize);
    depthBuffer.setStorage(GL::RenderbufferFormat::DepthComponent24, framebufferSize);

    framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer);
    framebuffer.attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, depthBuffer);
    framebuffer.mapForDraw({{Shaders::Phong::ColorOutput, GL::Framebuffer::ColorAttachment{0}}});

    framebuffer.clearColor(0, Color3{0.125f}).clearDepth(1.0).bind();

    CORRADE_INTERNAL_ASSERT(framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    for (int i = 0; i < env.numAgents; ++i) {
        agentFrames.emplace_back(size_t(framebufferSize.x() * framebufferSize.y() * 4));
        agentImageViews.emplace_back(std::make_unique<MutableImageView2D>(PixelFormat::RGBA8Unorm, framebufferSize, agentFrames[i]));
    }

    shaderInstanced = Shaders::Phong{Shaders::Phong::Flag::InstancedTransformation};
    shaderInstanced.setAmbientColor(0x555555_rgbf).setDiffuseColor(0xbbbbbb_rgbf).setShininess(300).setLightPosition({0,4,2}).setLightColor(0xaaaaaa_rgbf);

    shader = Shaders::Phong{};

    // agents
    {
        agentMesh = MeshTools::compile(Primitives::capsule3DSolid(3, 3, 8, 1.0));
        agentEyeMesh = MeshTools::compile(Primitives::cubeSolid());

        const auto &startingPositions = env.agentStartingPositions;
        for (int i = 0; i < env.numAgents; ++i) {
            auto &agentObj = env.agents[i];
            auto pos = Vector3{startingPositions[i]} + Vector3{0.5, 3.525, 0.5};
            agentObj->rotateY(frand(env.getRng()) * 360.0_degf);
            agentObj->translate(pos);

            auto agentDrawable = std::make_unique<SimpleDrawable3D>(*agentObj, drawables, shader, agentMesh,
                                                                    0xf9d71c_rgbf);
            agentDrawable->scale({0.25f, 0.25f * 0.9f, 0.25f});

            auto agentEyeDrawable = std::make_unique<SimpleDrawable3D>(*agentObj, drawables, shader, agentEyeMesh,
                                                                       0x222222_rgbf);
            agentEyeDrawable->scale({0.17, 0.075, 0.17}).translate({0.0f, 0.2f, -0.08f});

            agentDrawables.emplace_back(std::move(agentDrawable));
            agentDrawables.emplace_back(std::move(agentEyeDrawable));
        }
    }

    // exit pad
    {
        exitPadMesh = MeshTools::compile(Primitives::cubeSolid());
        const auto exitPadCoords = env.exitPad;
        const auto exitPadScale = Vector3(
            exitPadCoords.max.x() - exitPadCoords.min.x(),
            1.0,
            exitPadCoords.max.z() - exitPadCoords.min.z()
        );
        const auto exitPadPos = Vector3(exitPadCoords.min.x() + exitPadScale.x() / 2, exitPadCoords.min.y(),
                                        exitPadCoords.min.z() + exitPadScale.z() / 2);

        exitPadDrawable = std::make_unique<SimpleDrawable3D>(env.scene, drawables, shader, exitPadMesh, 0x50c878_rgbf);
        exitPadDrawable->scale({0.5, 0.025, 0.5}).scale(exitPadScale);
        exitPadDrawable->translate({0.0, 0.025, 0.0});
        exitPadDrawable->translate(exitPadPos);
    }

    // map layout
    {
        cubeMesh = MeshTools::compile(Primitives::cubeSolid());
        voxelInstanceBuffer = GL::Buffer{};
        cubeMesh.addVertexBufferInstanced(
            voxelInstanceBuffer, 1, 0,
            Shaders::Phong::TransformationMatrix{},
            Shaders::Phong::NormalMatrix{},
            Shaders::Phong::Color3{}
        );

        TLOG(INFO) << "Rendering " << env.layoutDrawables.size() << " layout drawables";

        for (auto layoutDrawable : env.layoutDrawables) {
            auto voxelObject = std::make_unique<Object3D>(&env.scene);

            const auto bboxMin = layoutDrawable.min, bboxMax = layoutDrawable.max;
            auto scale = Vector3{
                float(bboxMax.x() - bboxMin.x() + 1) / 2,
                float(bboxMax.y() - bboxMin.y() + 1) / 2,
                float(bboxMax.z() - bboxMin.z() + 1) / 2,
            };

            voxelObject->scale(scale).translate({0.5, 0.5, 0.5}).translate({
                float((bboxMin.x() + bboxMax.x())) / 2,
                float((bboxMin.y() + bboxMax.y())) / 2,
                float((bboxMin.z() + bboxMax.z())) / 2
            });

            auto transformation = Matrix4::scaling(Vector3{1.0f});

            auto voxel = std::make_unique<CustomDrawable>(
                *voxelObject, voxelInstanceData, 0xa5c9ea_rgbf, transformation, drawables
            );

            layoutObjects.emplace_back(std::move(voxelObject));
            instancedLayoutDrawables.emplace_back(std::move(voxel));
        }
    }
}

void MagnumEnvRenderer::Impl::draw(Env &env)
{
    for (int i = 0; i < env.numAgents; ++i) {
        framebuffer
            .clearColor(0, Color3{0.125f})
            .clearDepth(1.0f)
            .bind();

        arrayResize(voxelInstanceData, 0);

        auto activeCameraPtr = env.agents[i]->camera.get();

        activeCameraPtr->draw(drawables);

        shaderInstanced.setProjectionMatrix(activeCameraPtr->projectionMatrix());

        /* Upload instance data to the GPU (orphaning the previous buffer
           contents) and draw all cubes in one call  */
        voxelInstanceBuffer.setData(voxelInstanceData, GL::BufferUsage::DynamicDraw);
        cubeMesh.setInstanceCount(Int(voxelInstanceData.size()));
        shaderInstanced.draw(cubeMesh);

        framebuffer
            .mapForRead(GL::Framebuffer::ColorAttachment{0})
            .read(framebuffer.viewport(), *agentImageViews[i]);
    }
}

const uint8_t * MagnumEnvRenderer::Impl::getObservation(int agentIdx) const
{
    return agentFrames[agentIdx].data();
}

MagnumEnvRenderer::MagnumEnvRenderer(Env &env, int w, int h)
{
    // TODO: just create a windowless context here explicitly, no need to derive from WindowlessApplication
    int fakeArgc = 1;
    char *fakeArgv[1] = {"test"};
    Platform::WindowlessApplication::Arguments args{fakeArgc, &fakeArgv[0]};

    pimpl = std::make_unique<Impl>(args, env, w, h);
}

MagnumEnvRenderer::~MagnumEnvRenderer() = default;


void MagnumEnvRenderer::draw(Env &env)
{
    pimpl->draw(env);
}

const uint8_t * MagnumEnvRenderer::getObservation(int agentIdx) const
{
    return pimpl->getObservation(agentIdx);
}
