#pragma once

#include <env/env.hpp>


namespace VoxelWorld
{

class EnvRenderer
{
public:
    virtual ~EnvRenderer() = default;

    virtual void reset(Env &env, int envIdx) = 0;

    virtual void preDraw(Env &env, int envIndex) = 0;

    virtual void draw(Envs &envs) = 0;

    /**
     * Query the pointer to memory holding the latest observation for an agent in an env.
     * @param envIdx env index.
     * @param agentIdx agent for which we're querying the observation.
     * @return pointer to the memory holding the observation.
     */
    virtual const uint8_t *getObservation(int envIdx, int agentIdx) const = 0;
};

}