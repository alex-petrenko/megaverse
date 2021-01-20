#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_hexagonal_maze.hpp>

namespace VoxelWorld
{

class HexExploreScenario : public DefaultScenario
{
public:
    explicit HexExploreScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~HexExploreScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    [[nodiscard]] float trueObjective(int) const override { return solved; }

    RewardShaping defaultRewardShaping() const override
    {
        return {{Str::exploreSolved, 5.0f}};
    }

private:
    bool solved = false;

    HexagonalMazeComponent maze;

    Magnum::Vector3 rewardObjectCoords;
};

}