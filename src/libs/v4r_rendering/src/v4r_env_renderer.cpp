#include <v4r.hpp>
#include <v4r/debug.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <v4r_rendering/v4r_env_renderer.hpp>

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <util/tiny_logger.hpp>

#include <cuda.h>
#include <cuda_runtime.h>

#include <memory>
#include <vector>

using namespace std;
using namespace Magnum;
using namespace Magnum::Math::Literals;

class V4RDrawable : public SceneGraph::Drawable3D
{
public:
    explicit V4RDrawable(
            SceneGraph::AbstractObject3D &parentObject,
            v4r::Environment &renderEnv,
            uint32_t instanceID,
            SceneGraph::DrawableGroup3D &drawables)
        : SceneGraph::Drawable3D{parentObject, &drawables},
          _renderEnv(renderEnv),
          _instanceID(instanceID)
    {}

private:
    void draw(const Matrix4 &transformation, SceneGraph::Camera3D &) override
    {
        _renderEnv.updateInstanceTransform(_instanceID, glm::make_mat4(transformation.data()));
    }

    v4r::Environment &_renderEnv;
    uint32_t _instanceID;
};


struct ColorCompare
{
    bool operator()(const Magnum::Color3 &c1, const Magnum::Color3 &c2) const
    {
        return c1.r() == c2.r() ? c1.g() == c2.g() ? c1.b() < c2.b() : c1.g() < c2.g() : c1.r() < c2.r();
    }
};


struct V4REnvRenderer::Impl
{
public:
    explicit Impl(Env &env, int w, int h);

    ~Impl();

    /**
     * Reset the state of the renderer between episodes.
     * @param env
     */
    void reset(Env &env);

    void draw(Env &env);

    const uint8_t * getObservation(int agentIdx) const;

public:
    v4r::BatchRenderer renderer;
    v4r::AssetLoader loader;
    v4r::CommandStream cmdStream;
    shared_ptr<v4r::Scene> scene;
    vector<v4r::Environment> envs;

    glm::u32vec2 framebufferSize;

//    vector<uint8_t> cpuFrames;

    SceneGraph::DrawableGroup3D drawables;

    std::map<Color3, int, ColorCompare> materialIndices;

    // Fake camera for only object transformations
    SceneGraph::Camera3D *fakeCamera;

    v4r::RenderDoc rdoc;
};

using Pipeline = v4r::BlinnPhong<v4r::RenderOutputs::Color,
                                 v4r::DataSource::Uniform,
                                 v4r::DataSource::Uniform,
                                 v4r::DataSource::Uniform>;

V4REnvRenderer::Impl::Impl(Env &env, int w, int h)
    : renderer({ 0, 1, 1,
                 static_cast<uint32_t>(env.getNumAgents()),
                 static_cast<uint32_t>(w),
                 static_cast<uint32_t>(h), 
                 glm::mat4(1.f) },
               v4r::RenderFeatures<Pipeline> {
                   v4r::RenderOptions::CpuSynchronization
               }),
      loader(renderer.makeLoader()),
      cmdStream(renderer.makeCommandStream()),
      envs(),
      framebufferSize(w, h),
//      cpuFrames(),
      drawables(),
      fakeCamera(),
      rdoc()
{
    // Need to reserve numAgents here so references remain stable
    envs.reserve(size_t(env.getNumAgents()));

//    cpuFrames = vector<uint8_t>(size_t(framebufferSize.x * framebufferSize.y * 4 * env.getNumAgents()));

    vector<shared_ptr<v4r::Mesh>> meshes;
    vector<shared_ptr<v4r::Material>> materials;

    using Vertex = Pipeline::Vertex;
    using MaterialParams = Pipeline::MaterialParams;

    // Inefficient conversion
    auto convertMesh = [&](const Magnum::Trade::MeshData &magnum_mesh) {
        vector<Vertex> vertices;
        vector<uint32_t> indices;
        const auto magnum_indices = magnum_mesh.indicesAsArray();
        const auto magnum_positions = magnum_mesh.positions3DAsArray();
        const auto magnum_normals = magnum_mesh.normalsAsArray();

        for (size_t i = 0; i < magnum_positions.size(); i++) {
            const auto &position = magnum_positions[i];
            const auto &normal = magnum_normals[i];

            vertices.emplace_back(Vertex {
                glm::make_vec3(position.data()),
                glm::make_vec3(normal.data())
            });
        }

        for (uint32_t idx : magnum_indices) {
            indices.push_back(idx);
        }

        return loader.loadMesh(move(vertices), move(indices));
    };

    // meshes
    {
        auto capsuleMesh = Primitives::capsule3DSolid(3, 3, 8, 1.0);
        meshes.emplace_back(convertMesh(capsuleMesh));

        auto cubeMesh = Primitives::cubeSolid();
        meshes.emplace_back(convertMesh(cubeMesh));
    }

    // Materials
    {
        const auto palette = env.getPalette();
        constexpr float shininess = 300.0f;

        for (auto c : palette) {
            materialIndices[c] = int(materials.size());

            materials.emplace_back(loader.makeMaterial(MaterialParams {
                glm::vec3(c.r(), c.g(), c.b()),
                glm::vec3(1.f),
                shininess
            }));
        }
    }

    // Scene
    { 
        v4r::SceneDescription scene_desc(move(meshes), move(materials));
        scene_desc.addLight(glm::vec3(0, 4, 2), glm::vec3(0.66f));

        scene = loader.makeScene(scene_desc);
    }
}

V4REnvRenderer::Impl::~Impl()
{
    TLOG(INFO) << __PRETTY_FUNCTION__;
}

void V4REnvRenderer::Impl::reset(Env &env)
{
    fakeCamera = &env.scene->addFeature<SceneGraph::Camera3D>();

    envs.clear();
    for (int i = 0; i < env.getNumAgents(); i++) envs.emplace_back(cmdStream.makeEnvironment(scene, 115.f, 0.01f, 100.0f));

    // reset renderer data structures
    {
        drawables = SceneGraph::DrawableGroup3D{};
    }

    // drawables
    {
        constexpr auto capsuleMeshIdx = 0, boxMeshIdx = 1;

        for (int agentIdx = 0; agentIdx < env.getNumAgents(); ++agentIdx) {
            auto &renderEnv = envs[agentIdx];

            for (const auto &sceneObjectInfo : env.drawables[DrawableType::Capsule]) {
                const auto &color = sceneObjectInfo.color;
                const auto materialIdx = materialIndices[color];
                const auto renderID = renderEnv.addInstance(capsuleMeshIdx, uint32_t(materialIdx), glm::mat4(1.f));
                sceneObjectInfo.objectPtr->addFeature<V4RDrawable>(renderEnv, renderID, drawables);
            }

            for (const auto &sceneObjectInfo : env.drawables[DrawableType::Box]) {
                const auto &color = sceneObjectInfo.color;
                const auto materialIdx = materialIndices[color];
                const auto renderID = renderEnv.addInstance(boxMeshIdx, uint32_t(materialIdx), glm::mat4(1.f));
                sceneObjectInfo.objectPtr->addFeature<V4RDrawable>(renderEnv, renderID, drawables);
            }
        }
    }
}

void V4REnvRenderer::Impl::draw(Env &env)
{
    for (int i = 0; i < env.getNumAgents(); ++i) {
        v4r::Environment &renderEnv = envs[i];

        auto activeCameraPtr = env.agents[i]->camera;
        auto view = glm::make_mat4(activeCameraPtr->cameraMatrix().data());
        renderEnv.setCameraView(view);
    }

    fakeCamera->draw(drawables);

//    rdoc.startFrame();
    cmdStream.render(envs);
    cmdStream.waitForFrame();

    const auto remainingTimeBarThickness = 2;
    const auto numPixelsInOneRow = framebufferSize.x * 4;
    const auto pixelsToFill = env.remainingTimeFraction() * numPixelsInOneRow;

    for (int agentIdx = 0; agentIdx < env.getNumAgents(); ++agentIdx) {
        for (int i = 0; i < remainingTimeBarThickness; ++i) {
            // viewport is flipped upside-down
            const auto rowStart = numPixelsInOneRow * i;
            memset((void *)(cmdStream.getRGB() + agentIdx * framebufferSize.x * framebufferSize.y * 4 + rowStart), 255, size_t(pixelsToFill));
        }
    }

//    memcpy(
//        cpuFrames.data(),
//        cmdStream.getRGB(),
//        env.getNumAgents() * framebufferSize.x * framebufferSize.y * 4
//    );

//    rdoc.endFrame();

//    cudaError_t cuda_res = cudaStreamSynchronize(cudaStream);
//    if (cuda_res != cudaSuccess)
//        abort();
}

const uint8_t * V4REnvRenderer::Impl::getObservation(int agentIdx) const
{
    return cmdStream.getRGB() + agentIdx * framebufferSize.x * framebufferSize.y * 4;
//    return cpuFrames.data() + agentIdx * framebufferSize.x * framebufferSize.y * 4;
}

V4REnvRenderer::V4REnvRenderer(Env &env, int w, int h)
{
    pimpl = std::make_unique<Impl>(env, w, h);
}

V4REnvRenderer::~V4REnvRenderer() = default;


void V4REnvRenderer::reset(Env &env)
{
    pimpl->reset(env);
}

void V4REnvRenderer::draw(Env &env)
{
    pimpl->draw(env);
}

const uint8_t * V4REnvRenderer::getObservation(int agentIdx) const
{
    return pimpl->getObservation(agentIdx);
}
