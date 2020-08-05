#pragma once

#include <memory>

#include <env/env_renderer.hpp>

class V4REnvRenderer : public EnvRenderer
{
public:
    explicit V4REnvRenderer(Env &env, int w, int h);
    ~V4REnvRenderer() override;

    void reset(Env &env) override;

    void draw(Env &env) override;

    const uint8_t * getObservation(int agentIdx) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};
