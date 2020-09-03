#pragma once

#include <memory>

#include <Magnum/GL/Renderbuffer.h>

#include <util/magnum.hpp>

#include <env/env_renderer.hpp>
#include <magnum_rendering/windowless_context.hpp>



class MagnumEnvRenderer : public EnvRenderer
{
public:
    explicit MagnumEnvRenderer(Env &env, int w, int h, bool withDebugDraw = false, RenderingContext *ctx = nullptr);
    ~MagnumEnvRenderer() override;

    void reset(Env &env) override;

    void draw(Env &env) override;
    void drawAgent(Env &env, int agentIndex, bool readToBuffer);

    const uint8_t * getObservation(int agentIdx) const override;

    Magnum::GL::Framebuffer * getFramebuffer();

    void toggleOverviewMode();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};