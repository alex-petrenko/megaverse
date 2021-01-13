#pragma once

#include <memory>

#include <Magnum/GL/Renderbuffer.h>

#include <util/magnum.hpp>

#include <env/vector_env.hpp>
#include <env/env_renderer.hpp>

#include <magnum_rendering/windowless_context.hpp>


namespace VoxelWorld
{

struct Overview
{
public:
    void saveTransformation()
    {
        rootTransformation = root->transformation();
        verticalTiltTransformation = verticalTilt->transformation();
    }

    void restoreTransformation()
    {
        root->setTransformation(rootTransformation);
        verticalTilt->setTransformation(verticalTiltTransformation);
    }

public:
    // TODO: we can use only one object, but this works for now
    Object3D *root{}, *verticalTilt{};

    Magnum::SceneGraph::Camera3D *camera{};
    bool enabled = false;

    float verticalRotation = 0.0f;

    Magnum::Matrix4 rootTransformation{}, verticalTiltTransformation{};
};

class MagnumEnvRenderer : public EnvRenderer
{
public:
    explicit MagnumEnvRenderer(
        Envs &envs, int w, int h, bool withDebugDraw = false, bool withOverview = false, RenderingContext *ctx = nullptr
    );

    ~MagnumEnvRenderer() override;

    void reset(Env &env, int envIdx) override;

    void preDraw(Env &env, int envIdx) override;

    void draw(Envs &envs) override;

    void drawAgent(Env &env, int envIdx, int agentIndex, bool readToBuffer);

    void postDraw(Env &env, int envIdx) override;

    const uint8_t * getObservation(int envIdx, int agentIdx) const override;

    Magnum::GL::Framebuffer *getFramebuffer();

    void toggleDebugMode();

    Overview & getOverview();


private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}