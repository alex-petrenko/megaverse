#pragma once

#include <memory>

#include <util/magnum.hpp>

#include <env/env_renderer.hpp>



class MagnumEnvRenderer : public EnvRenderer
{
public:
    explicit MagnumEnvRenderer(Env &env, int w, int h);
    ~MagnumEnvRenderer() override;

    virtual void draw(Env &env) override;

    virtual const uint8_t * getObservation(int agentIdx) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};