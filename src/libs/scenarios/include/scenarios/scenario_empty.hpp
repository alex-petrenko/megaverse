#pragma once

#include <scenarios/scenario_default.hpp>


namespace VoxelWorld
{

class EmptyScenario : public DefaultScenario
{
public:
    explicit EmptyScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~EmptyScenario() override;

    // Scenario interface
    void reset() override;

    void step() override {}

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int) const override { return 0; }

    RewardShaping defaultRewardShaping() const override { return {}; }
};

}