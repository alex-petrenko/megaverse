#pragma once

#include <memory>

#include <Magnum/GL/Renderbuffer.h>

#include <util/magnum.hpp>

#include <env/vector_env.hpp>
#include <env/env_renderer.hpp>

#include <magnum_rendering/windowless_context.hpp>


namespace VoxelWorld
{

class MagnumEnvRenderer : public EnvRenderer
{
public:
    explicit MagnumEnvRenderer(
        Envs &envs, int w, int h, bool withDebugDraw = false, RenderingContext *ctx = nullptr
    );

    ~MagnumEnvRenderer() override;

    void reset(Env &env, int envIdx) override;

    void preDraw(Env &env, int envIdx) override;

    void draw(Envs &envs) override;

    void drawAgent(Env &env, int envIdx, int agentIndex, bool readToBuffer);

    void postDraw(Env &env, int envIdx) override;

    const uint8_t *getObservation(int envIdx, int agentIdx) const override;

    Magnum::GL::Framebuffer *getFramebuffer();

    void toggleOverviewMode();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}