#pragma once

#include <env/env.hpp>


class EnvRenderer
{
public:
    virtual ~EnvRenderer() = default;

    virtual void draw(Env &env) = 0;

    virtual const uint8_t * getObservation(int agentIdx) const = 0;
};