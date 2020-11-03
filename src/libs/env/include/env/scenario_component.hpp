#pragma once

#include <env/scenario.hpp>


namespace VoxelWorld
{

/**
 * We decompose behaviors in scenarios into reusable components which can be used in multiple scenarios.
 * This is a base class for all scenario components.
 */
class ScenarioComponent
{
public:
    explicit ScenarioComponent(Scenario &scenario)
        : scenario{scenario}
    {
    }

    virtual ~ScenarioComponent() = default;

public:
    virtual void reset(Env &, Env::EnvState &) {}

    virtual void step(Env &, Env::EnvState &) {}

protected:
    /**
     * When needed a component can access the parent scenario.
     */
    Scenario &scenario;
};

}