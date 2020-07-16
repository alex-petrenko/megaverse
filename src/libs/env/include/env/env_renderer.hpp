#pragma once

#include <env/env.hpp>


class EnvRenderer
{
public:
    virtual ~EnvRenderer() = default;

    /**
     * Reset the renderer state on the episode boundary.
     */
    virtual void reset(Env &env) = 0;

    /**
     * Render the agent views and store results in the buffers internally.
     */
    virtual void draw(Env &env) = 0;

    /**
     * Query the pointer to memory holding the latest observation for an agent.
     * @param agentIdx agent for which we're querying the observation.
     * @return pointer to the memory holding the observation.
     */
    virtual const uint8_t * getObservation(int agentIdx) const = 0;
};