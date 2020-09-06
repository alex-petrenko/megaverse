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
    void draw(const Matrix4 &transformation,
              SceneGraph::Camera3D &) override {
        _renderEnv.updateInstanceTransform(_instanceID,
            glm::make_mat4(transformation.data()));
    }

    v4r::Environment &_renderEnv;
    uint32_t _instanceID;
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

    vector<vector<uint8_t>> cpuFrames;

    SceneGraph::DrawableGroup3D drawables;

    // Convenience variables for mesh / material indices
    const uint32_t agentIdx { 0 };
    const uint32_t eyeIdx { 1 };
    const uint32_t exitIdx { 2 };
    const uint32_t cubeIdx { 3 };

    // Fake camera for only object transformations
    SceneGraph::Camera3D *fakeCamera;

    cudaStream_t cudaStream;

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
      cpuFrames(),
      drawables(),
      fakeCamera(),
      cudaStream(),
      rdoc()
{
    cudaError_t cuda_res = cudaStreamCreate(&cudaStream);
    if (cuda_res != cudaSuccess) {
        abort();
    }
    // Need to reserve numAgents here so references remain stable
    envs.reserve(env.getNumAgents());

    for (int i = 0; i < env.getNumAgents(); ++i) {
        cpuFrames.emplace_back(size_t(framebufferSize.x *
                                      framebufferSize.y * 4));
    }

    vector<shared_ptr<v4r::Mesh>> meshes;
    vector<shared_ptr<v4r::Material>> materials;

    using Vertex = Pipeline::Vertex;
    using MaterialParams = Pipeline::MaterialParams;

    // Inefficient conversion
    auto convertMesh = [&](const Magnum::Trade::MeshData &magnum_mesh) {
        vector<Vertex> vertices;
        vector<uint32_t> indices;
        const auto magnum_indices = magnum_mesh.indicesAsArray();
        const auto magnum_positions =
            magnum_mesh.positions3DAsArray();
        const auto magnum_normals =
            magnum_mesh.normalsAsArray();

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
        auto agentMesh = Primitives::capsule3DSolid(3, 3, 8, 1.0);
        meshes.emplace_back(convertMesh(agentMesh));

        auto agentEyeMesh = Primitives::cubeSolid();
        meshes.emplace_back(convertMesh(agentEyeMesh));

        auto exitPadMesh = Primitives::cubeSolid();
        meshes.emplace_back(convertMesh(exitPadMesh));

        auto cubeMesh = Primitives::cubeSolid();
        meshes.emplace_back(convertMesh(cubeMesh));
    }

    // Materials
    {
        // Agent material
        materials.emplace_back(loader.makeMaterial(MaterialParams {
            glm::vec3(0.976f, 0.843f, 0.110f),
            glm::vec3(1.f),
            150.f
        }));

        // Eye material
        materials.emplace_back(loader.makeMaterial(MaterialParams {
            glm::vec3(0.133f),
            glm::vec3(1.f),
            150.f
        }));

        // Exit pad material
        materials.emplace_back(loader.makeMaterial(MaterialParams {
            glm::vec3(0.314f, 0.784f, 0.471f),
            glm::vec3(1.f),
            150.f
        }));

        // Voxel material
        materials.emplace_back(loader.makeMaterial(MaterialParams {
            glm::vec3(0.647f, 0.788f, 0.918f),
            glm::vec3(1.f),
            300.f
        }));
    }

    // Scene
    { 
        v4r::SceneDescription scene_desc(move(meshes), move(materials));
        scene_desc.addLight(glm::vec3(2.f, 10.f, 2.f),
                            glm::vec3(0.667f));

        scene = loader.makeScene(scene_desc);
    }
}

V4REnvRenderer::Impl::~Impl()
{
    cudaStreamDestroy(cudaStream);
    TLOG(INFO) << __PRETTY_FUNCTION__;
}

void V4REnvRenderer::Impl::reset(Env &env)
{
    fakeCamera = &env.scene->addFeature<SceneGraph::Camera3D>();

    envs.clear();
    for (int i = 0; i < env.getNumAgents(); i++) {
        envs.emplace_back(cmdStream.makeEnvironment(scene,
            75.f));
    }

    // reset renderer data structures
    {
        drawables = SceneGraph::DrawableGroup3D{};
    }

    // agents
    {
        const auto &startingPositions = env.agentStartingPositions;
        for (int i = 0; i < env.getNumAgents(); ++i) {
            auto agentPtr = env.agents[i];
            auto pos = Vector3{startingPositions[i]} + Vector3{0.5, 3.525, 0.5};
            agentPtr->rotateY(frand(env.getRng()) * 360.0_degf);
            agentPtr->translate(pos);

            auto &agentObject = agentPtr->addChild<Object3D>();
            auto &agentEyeObject = agentPtr->addChild<Object3D>();
            agentObject.scale({0.25f, 0.25f * 0.9f, 0.25f});
            agentEyeObject.scale({0.17, 0.075, 0.17}).
                translate({0.0f, 0.2f, -0.08f});

            for (int j = 0; j < env.getNumAgents(); j++) {
                // Add instance of agent (meshidx & material_idx = agentIdx)
                // Transform doesn't matter as it will be overwritten when
                // draw is called.
                v4r::Environment &renderEnv = envs[j];

                uint32_t agentRenderID =
                    renderEnv.addInstance(agentIdx, agentIdx, glm::mat4(1.f));

                uint32_t eyeRenderID =
                    renderEnv.addInstance(eyeIdx, eyeIdx, glm::mat4(1.f));

                agentObject.addFeature<V4RDrawable>(
                        renderEnv, agentRenderID, drawables);

                agentEyeObject.addFeature<V4RDrawable>(
                        renderEnv, eyeRenderID, drawables);
            }
        }
    }

    // exit pad
    {
        const auto exitPadCoords = env.exitPad;
        const auto exitPadScale = Vector3(
            exitPadCoords.max.x() - exitPadCoords.min.x(),
            1.0,
            exitPadCoords.max.z() - exitPadCoords.min.z()
        );
        const auto exitPadPos = Vector3(exitPadCoords.min.x() + exitPadScale.x() / 2, exitPadCoords.min.y(),
                                        exitPadCoords.min.z() + exitPadScale.z() / 2);

        auto &exitPadObject = env.scene->addChild<Object3D>();

        exitPadObject.scale({0.5, 0.025, 0.5}).scale(exitPadScale);
        exitPadObject.translate({0.0, 0.025, 0.0});
        exitPadObject.translate(exitPadPos);

        for (int i = 0; i < env.getNumAgents(); i++) {
            v4r::Environment &renderEnv = envs[i];
            uint32_t exitRenderID =
                renderEnv.addInstance(exitIdx, exitIdx, glm::mat4(1.f));

            exitPadObject.addFeature<V4RDrawable>(
                renderEnv, exitRenderID, drawables);
        }
    }

    // map layout
    {
        TLOG(INFO) << "Rendering " << env.layoutDrawables.size() << " layout drawables";

        for (auto layoutDrawable : env.layoutDrawables) {
            auto &voxelObject = env.scene->addChild<Object3D>();

            const auto bboxMin = layoutDrawable.min, bboxMax = layoutDrawable.max;
            auto scale = Vector3{
                float(bboxMax.x() - bboxMin.x() + 1) / 2,
                float(bboxMax.y() - bboxMin.y() + 1) / 2,
                float(bboxMax.z() - bboxMin.z() + 1) / 2,
            };

            voxelObject.scale(scale)
                .translate({0.5, 0.5, 0.5})
                .translate({float((bboxMin.x() + bboxMax.x())) / 2, float((bboxMin.y() + bboxMax.y())) / 2, float((bboxMin.z() + bboxMax.z())) / 2});

            //auto transformation = Matrix4::scaling(Vector3{1.0f});

            for (int i= 0; i < env.getNumAgents(); i++) {
                v4r::Environment &renderEnv = envs[i];

                uint32_t voxelID =
                    renderEnv.addInstance(cubeIdx, cubeIdx, glm::mat4(1.f));

                voxelObject.addFeature<V4RDrawable>(
                        renderEnv, voxelID, drawables);
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

    rdoc.startFrame();
    cmdStream.render(envs);
    cmdStream.waitForFrame();
    rdoc.endFrame();

    // for (int i = 0; i < env.getNumAgents(); i++)
    //    memcpy(cpuFrames[i].data(), cmdStream.getRGB() + i * framebufferSize.x * framebufferSize.y * 4, framebufferSize.x * framebufferSize.y * 4);

    cudaError_t cuda_res = cudaStreamSynchronize(cudaStream);
    if (cuda_res != cudaSuccess) {
        abort();
    }
}

const uint8_t * V4REnvRenderer::Impl::getObservation(int agentIdx) const
{
    return cmdStream.getRGB() + agentIdx * framebufferSize.x * framebufferSize.y * 4;
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
