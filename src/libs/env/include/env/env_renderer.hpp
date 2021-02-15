#pragma once

#include <env/env.hpp>


namespace VoxelWorld
{

// defined later in render_utils.cpp
class Overview;

class EnvRenderer
{
public:
    virtual ~EnvRenderer() = default;

    virtual void reset(Env &env, int envIdx) = 0;

    virtual void preDraw(Env &env, int envIndex) = 0;

    virtual void draw(Envs &envs) = 0;

    virtual bool isVulkan() const { return false; }

    /**
     * Query the pointer to memory holding the latest observation for an agent in an env.
     * @param envIdx env index.
     * @param agentIdx agent for which we're querying the observation.
     * @return pointer to the memory holding the observation.
     */
    virtual const uint8_t *getObservation(int envIdx, int agentIdx) const = 0;

    virtual Overview * getOverview() = 0;
};

inline std::tuple<float, float, float, float> agentCameraParameters()
{
    float fov = 100, near = 0.01, far = 120.0, aspectRatio = 128.0f / 72.0f;
    return std::make_tuple(fov, near, far, aspectRatio);
}

inline std::tuple<float, float, float, float> overviewCameraParameters()
{
    auto [fov, near, far, aspectRatio] = agentCameraParameters();
    fov = 100, near = 0.1, far = 600.0;

    return std::make_tuple(fov, near, far, aspectRatio);
}

}
