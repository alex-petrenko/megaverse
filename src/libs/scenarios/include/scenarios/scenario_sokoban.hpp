#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace Megaverse
{

struct SokobanLevel
{
    std::vector<std::string> rows;
};


class SokobanScenario : public DefaultScenario
{
public:
    explicit SokobanScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~SokobanScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    void reloadLevels();

    void createLayout();

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int) const override { return float(solved); }

    RewardShaping defaultRewardShaping() const override
    {
        return {
            {Str::sokobanBoxOnTarget, 1.0f},
            {Str::sokobanBoxLeavesTarget, -1.0f},
            {Str::sokobanAllBoxesOnTarget, 10.0f},
        };
    }

    void initializeDefaultParameters() override
    {
        DefaultScenario::initializeDefaultParameters();
        floatParams[Str::episodeLengthSec] = 80.0f;
    }

private:
    std::string boxobanLevelsDir{};
    std::vector<std::string> allSokobanLevelFiles{};
    constexpr static ConstStr levelSet = "unfiltered", levelSplit = "train";

    std::vector<SokobanLevel> levels;
    SokobanLevel currLevel;
    int length = 0, width = 0;

    VoxelGridComponent<VoxelWithPhysicsObjects> vg;
    const float voxelSize = 2;

    std::vector<Magnum::Vector3> agentPositions;
    std::vector<VoxelCoords> boxesCoords;
    int numBoxes = 0, numBoxesOnGoal = 0;

    bool solved = false;
};

}