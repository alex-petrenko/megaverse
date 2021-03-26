#pragma once

#include <memory>

#include <env/env_renderer.hpp>


namespace VoxelWorld
{

class V4RDrawable;

class V4REnvRenderer : public EnvRenderer
{
public:
    /**
     * @param previousRenderer if renderers are chained (i.e. multiple renderers render the same scene) we need the
     * pointer to the previous renderer in the chain to provide the list of "dirty" drawables whose absolute
     * transformations we need to query from the scene graph and update.
     */
    explicit V4REnvRenderer(Envs &envs, int w, int h, V4REnvRenderer *previousRenderer, bool withOverview);

    ~V4REnvRenderer() override;

    void reset(Env &env, int envIdx) override;

    void preDraw(Env &env, int envIndex) override;

    void draw(Envs &envs) override;

    bool supportsParallelReset() const override { return true; }

    const uint8_t * getObservation(int envIdx, int agentIdx) const override;

    std::vector<int> getDirtyDrawables(int envIdx) const;

    Overview * getOverview() override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
