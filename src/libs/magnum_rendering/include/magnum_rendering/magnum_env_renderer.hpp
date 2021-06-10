#pragma once

#include <memory>

#include <Magnum/GL/Renderbuffer.h>

#include <util/magnum.hpp>

#include <env/vector_env.hpp>
#include <env/env_renderer.hpp>

#include <rendering/render_utils.hpp>

#include <magnum_rendering/rendering_context.hpp>


namespace Megaverse
{

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

    const uint8_t * getObservation(int envIdx, int agentIdx) const override;

    Magnum::GL::Framebuffer *getFramebuffer();

    void toggleDebugMode();

    Overview * getOverview() override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}