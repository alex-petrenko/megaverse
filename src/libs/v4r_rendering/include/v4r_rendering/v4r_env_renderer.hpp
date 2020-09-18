#pragma once

#include <memory>

#include <env/env_renderer.hpp>

class V4REnvRenderer : public EnvRenderer
{
public:
    explicit V4REnvRenderer(Envs &envs, int w, int h);
    ~V4REnvRenderer() override;

    void reset(Env &env, int envIdx) override;
    void preDraw(Env &env, int envIndex) override;
    void draw(Envs &envs) override;
    void postDraw(Env &env, int envIndex) override;

    const uint8_t * getObservation(int envIdx, int agentIdx) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};
