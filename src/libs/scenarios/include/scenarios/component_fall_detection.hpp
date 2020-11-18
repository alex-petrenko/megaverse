#pragma once

#include <env/scenario_component.hpp>


namespace VoxelWorld
{

class FallDetectionCallbacks
{
public:
    virtual void agentFell(int /*agentIdx*/) {}
};

template<typename VoxelT>
class FallDetectionComponent : public ScenarioComponent
{
public:
    explicit FallDetectionComponent(Scenario &scenario, VoxelGrid<VoxelT> &grid, FallDetectionCallbacks &callbacks, int fallThreshold = -20)
    : ScenarioComponent{scenario}
    , grid{grid}
    , callbacks{callbacks}
    , fallThreshold{fallThreshold}
    {
    }

    void reset(Env &, Env::EnvState &) override
    {
        agentInitialPositions.clear();
    }

    void step(Env &env, Env::EnvState &envState) override
    {
        for (int i = 0; i < env.getNumAgents(); ++i) {
            auto &a = envState.agents[i];
            if (a->absoluteTransformation().translation().y() < fallThreshold) {
                resetAgent(i, a);
                callbacks.agentFell(i);
            }
        }
    }

    void resetAgent(int agentIdx, AbstractAgent *a)
    {
        auto p = agentInitialPositions[agentIdx];
        auto v = grid.getWithVector(p);
        while (v && !v->empty() && p.y() < 1000) {
            ++p.y();
            v = grid.getWithVector(p);
        }

        const float halfVoxel = grid.getVoxelSize() / 2;
        a->teleport(btVector3{p.x() + halfVoxel, p.y() + halfVoxel, p.z() + halfVoxel});
    }

public:
    VoxelGrid<VoxelT> &grid;
    std::vector<Magnum::Vector3> agentInitialPositions;
    FallDetectionCallbacks &callbacks;
    int fallThreshold = -20;
};

}