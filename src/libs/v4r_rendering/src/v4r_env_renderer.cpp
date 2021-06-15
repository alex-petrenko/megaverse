#include <memory>
#include <vector>

#include <glm/ext.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/AbstractObject.h>

#include <cuda.h>
#include <cuda_runtime.h>

#include <v4r.hpp>
#include <v4r/debug.hpp>

#include <util/tiny_logger.hpp>

#include <rendering/render_utils.hpp>

#include <v4r_rendering/v4r_env_renderer.hpp>


using namespace std;
using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace Megaverse;


class Megaverse::V4RDrawable : public SceneGraph::Drawable3D
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

public:
    void updateAbsoluteTransformation()
    {
        _renderEnv.updateInstanceTransform(_instanceID, glm::make_mat4(object().absoluteTransformationMatrix().data()));
    }

private:
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
    explicit Impl(Envs &envs, int w, int h, V4REnvRenderer *previousRenderer, bool withOverview);

    ~Impl();

    /**
     * Reset the state of the renderer between episodes.
     * @param env
     */
    void reset(Env &env, int envIdx);

    void preDraw(Env &env, int envIdx);
    void draw(Envs &envs);

    const uint8_t * getObservation(int envIdx, int agentIdx) const;

    /**
     * Assuming preDraw() and draw() were already called for this renderer before the next renderer in the chain
     * requests dirty drawables.
     */
    std::vector<int> getDirtyDrawables(int envIdx) const { return dirtyDrawables[envIdx]; }

    Overview * getOverview() { return &overview; }

private:
    int batchSize(const Envs &envs) const
    {
        int res = 0;
        for (const auto &e : envs)
            res += e->getNumAgents();

        return res;
    }

public:
    v4r::BatchRenderer renderer;
    v4r::AssetLoader loader;
    v4r::CommandStream cmdStream;
    shared_ptr<v4r::Scene> scene;
    vector<v4r::Environment> renderEnvs;

    glm::u32vec2 framebufferSize;

    int pixelsPerFrame{}, pixelsPerEnv{};

//    vector<uint8_t> cpuFrames;

    std::vector<SceneGraph::DrawableGroup3D> envDrawables;
    std::vector<std::vector<V4RDrawable *>> v4rDrawables;  // to avoid dynamic cast on every step()
    std::vector<std::vector<std::reference_wrapper<SceneGraph::AbstractObject3D>>> drawablesObjects;

    std::vector<std::vector<int>> dirtyDrawables;

    std::map<Color3, int, ColorCompare> materialIndices;

//    v4r::RenderDoc rdoc;

    std::map<DrawableType, Trade::MeshData> meshData;
    std::map<DrawableType, int> meshIndices;

    V4REnvRenderer *previousRenderer = nullptr;

    bool withOverviewCamera = false;
    Overview overview;
};

using Pipeline = v4r::BlinnPhong<v4r::RenderOutputs::Color,
                                 v4r::DataSource::Uniform,
                                 v4r::DataSource::Uniform,
                                 v4r::DataSource::Uniform>;

V4REnvRenderer::Impl::Impl(Envs &envs, int w, int h, V4REnvRenderer *previousRenderer, bool withOverview)
    : renderer{{ 0, 1, 1, uint32_t(batchSize(envs)), uint32_t(w), uint32_t(h), glm::mat4(1.f) },
               v4r::RenderFeatures<Pipeline> {v4r::RenderOptions::CpuSynchronization}}
    , loader{renderer.makeLoader()}
    , cmdStream{renderer.makeCommandStream()}
    , renderEnvs{}
    , framebufferSize{w, h}
    , previousRenderer{previousRenderer}
    , withOverviewCamera{withOverview}
    // cpuFrames(),
    // rdoc()
{
    auto numEnvs = envs.size();
    envDrawables.resize(numEnvs), drawablesObjects.resize(numEnvs), v4rDrawables.resize(numEnvs), dirtyDrawables.resize(numEnvs);

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

        for (uint32_t idx : magnum_indices)
            indices.push_back(idx);

        return loader.loadMesh(move(vertices), move(indices));
    };

    // meshes
    {
        initPrimitives(meshData);
        for (const auto &[drawable, data] : meshData) {
            meshIndices[drawable] = int(meshes.size());
            meshes.emplace_back(convertMesh(data));
        }
    }

    // Materials
    {
        const auto palette = envs.front()->getPalette();
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

    // vector of render envs
    {
        auto [fov, near, far, aspectRatio] = agentCameraParameters();
        UNUSED(aspectRatio);

        for (auto &env : envs)
            for (int agentIdx = 0; agentIdx < env->getNumAgents(); ++agentIdx)
                renderEnvs.emplace_back(cmdStream.makeEnvironment(scene, fov, near, far));
    }

    pixelsPerFrame = framebufferSize.x * framebufferSize.y * 4;
    pixelsPerEnv = envs.front()->getNumAgents() * pixelsPerFrame;
}

V4REnvRenderer::Impl::~Impl()
{
    TLOG(INFO) << __PRETTY_FUNCTION__;
}

void V4REnvRenderer::Impl::reset(Env &env, int envIdx)
{
    auto [fov, near, far, aspectRatio] = agentCameraParameters();
    if (withOverviewCamera && envIdx == 0) {
        auto [oFov, oNear, oFar, oAspectRatio] = overviewCameraParameters();
        fov = oFov, near = oNear, far = oFar, aspectRatio = oAspectRatio;
    }
    UNUSED(aspectRatio);

    for (int i = 0; i < env.getNumAgents(); ++i) {
        const auto idx = envIdx * env.getNumAgents() + i;  // assuming all envs have the same numAgents
        renderEnvs[idx] = cmdStream.makeEnvironment(scene, fov, near, far);
    }

    // reset renderer data structures
    {
        envDrawables[envIdx] = SceneGraph::DrawableGroup3D{};
    }

    // drawables
    {
        const auto &drawables = env.getDrawables();

        for (int agentIdx = 0; agentIdx < env.getNumAgents(); ++agentIdx) {
            const auto renderEnvIdx = envIdx * env.getNumAgents() + agentIdx;
            auto &renderEnv = renderEnvs[renderEnvIdx];

            for (const auto &[drawableType, meshIndex] : meshIndices) {
                for (const auto &sceneObjectInfo : drawables.at(drawableType)) {
                    const auto &color = sceneObjectInfo.color;
                    const auto materialIdx = materialIndices[color];  // if we forgot to add the color to the palette, we should crash here
                    const auto renderID = renderEnv.addInstance(uint32_t(meshIndex), uint32_t(materialIdx), glm::mat4(1.f));
                    sceneObjectInfo.objectPtr->addFeature<V4RDrawable>(renderEnv, renderID, envDrawables[envIdx]);
                }
            }
        }

        drawablesObjects[envIdx].clear(), v4rDrawables[envIdx].clear();

        for (size_t i = 0; i < envDrawables[envIdx].size(); ++i) {
            auto &object = envDrawables[envIdx][i].object();
            drawablesObjects[envIdx].emplace_back(object);

            auto v4rDrawable = dynamic_cast<V4RDrawable *>(&envDrawables[envIdx][i]);
            v4rDrawables[envIdx].emplace_back(v4rDrawable);
        }

        dirtyDrawables[envIdx].clear();
    }

    // controllable overview camera
    if (withOverviewCamera && envIdx == 0)
        overview.reset(&env.getScene());
}

void V4REnvRenderer::Impl::preDraw(Env &env, int envIdx)
{
    dirtyDrawables[envIdx].clear();

    const auto numAgents = env.getNumAgents();
    for (int agentIdx = 0; agentIdx < numAgents; ++agentIdx) {
        const auto renderEnvIdx = envIdx * numAgents + agentIdx;
        v4r::Environment &renderEnv = renderEnvs[renderEnvIdx];

        auto activeCameraPtr = env.getAgents()[agentIdx]->getCamera();
        if (withOverviewCamera && overview.enabled && envIdx == 0)
            activeCameraPtr = overview.camera;

        auto view = glm::make_mat4(activeCameraPtr->cameraMatrix().data());

        renderEnv.setCameraView(view);
    }

    if (previousRenderer) {
        dirtyDrawables[envIdx] = previousRenderer->getDirtyDrawables(envIdx);
    } else {
        std::vector<std::reference_wrapper<SceneGraph::AbstractObject3D>> dirtyObjects;

        for (size_t i = 0; i < drawablesObjects[envIdx].size(); ++i) {
            auto &obj = drawablesObjects[envIdx][i].get();
            if (obj.isDirty()) {
                dirtyObjects.emplace_back(drawablesObjects[envIdx][i]);
                dirtyDrawables[envIdx].emplace_back(i);
            }
        }

        // Collapsing the scene graph transformations "manually", somehow this is barely faster, if at all
        SceneGraph::AbstractObject3D::setClean(dirtyObjects);
    }

    for (auto &drawableIdx : dirtyDrawables[envIdx])
        v4rDrawables[envIdx][drawableIdx]->updateAbsoluteTransformation();
}

void V4REnvRenderer::Impl::draw(Envs &)
{
//    rdoc.startFrame();
    cmdStream.render(renderEnvs);
    cmdStream.waitForFrame();

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

const uint8_t * V4REnvRenderer::Impl::getObservation(int envIdx, int agentIdx) const
{
    const auto startIdx = envIdx * pixelsPerEnv;
    return cmdStream.getRGB() + startIdx + agentIdx * pixelsPerFrame;
//    return cpuFrames.data() + agentIdx * framebufferSize.x * framebufferSize.y * 4;
}

V4REnvRenderer::V4REnvRenderer(Envs &envs, int w, int h, V4REnvRenderer *previousRenderer, bool withOverview)
{
    pimpl = std::make_unique<Impl>(envs, w, h, previousRenderer, withOverview);
}

V4REnvRenderer::~V4REnvRenderer() = default;


void V4REnvRenderer::reset(Env &env, int envIdx)
{
    pimpl->reset(env, envIdx);
}

void V4REnvRenderer::preDraw(Env &env, int envIndex)
{
    pimpl->preDraw(env, envIndex);
}

void V4REnvRenderer::draw(Envs &envs)
{
    pimpl->draw(envs);
}

const uint8_t * V4REnvRenderer::getObservation(int envIdx, int agentIdx) const
{
    return pimpl->getObservation(envIdx, agentIdx);
}

std::vector<int> V4REnvRenderer::getDirtyDrawables(int envIdx) const
{
    return pimpl->getDirtyDrawables(envIdx);
}

Overview * V4REnvRenderer::getOverview()
{
    return pimpl->getOverview();
}
