#pragma once

#include <scenarios/platforms.hpp>
#include <scenarios/scenario_default.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

struct VoxelObstacles : public VoxelWithPhysicsObjects
{
    Object3D *rewardObject = nullptr;
};

class ObstaclesScenario : public DefaultScenario, public ObjectStackingCallbacks, public FallDetectionCallbacks
{
public:
    explicit ObstaclesScenario(const std::string &name, Env &env, Env::EnvState &envState);

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override { return agentSpawnPositions; }

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int) const override { return solved; }

    RewardShaping defaultRewardShaping() const override
    {
        return {
            {Str::obstaclesAgentAtExit, 1.0f},
            {Str::obstaclesAllAgentsAtExit, 5.0f},
            {Str::obstacleExtraReward, 0.5f},
        };
    }

    float episodeLengthSec() const override;

    void agentTouchedLava(int agentIdx);

    void agentFell(int agentIdx) override;

private:
    VoxelGridComponent<VoxelObstacles> vg;
    PlatformsComponent platformsComponent;
    ObjectStackingComponent<VoxelObstacles> objectStackingComponent;
    FallDetectionComponent<VoxelObstacles> fallDetection;

    std::vector<VoxelCoords> objectSpawnPositions, rewardSpawnPositions;
    std::vector<Magnum::Vector3> agentSpawnPositions;

    std::vector<bool> agentReachedExit;
    bool solved = false;

    int numPlatforms{};
};

class ObstaclesEasyScenario : public ObstaclesScenario
{
public:
    explicit ObstaclesEasyScenario(const std::string &name, Env &env, Env::EnvState &envState)
    : ObstaclesScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesScenario::initializeDefaultParameters();

        auto &fp = floatParams;
        fp[Str::obstaclesMinNumPlatforms] = 1;
        fp[Str::obstaclesMaxNumPlatforms] = 2;

        fp[Str::obstaclesMinGap] = 1;
        fp[Str::obstaclesMaxGap] = 2;

        fp[Str::obstaclesMinLava] = 1;
        fp[Str::obstaclesMaxLava] = 4;

        fp[Str::obstaclesMinHeight] = 1;
        fp[Str::obstaclesMaxHeight] = 3;
    }
};

class ObstaclesMediumScenario : public ObstaclesScenario
{
public:
    explicit ObstaclesMediumScenario(const std::string &name, Env &env, Env::EnvState &envState)
    : ObstaclesScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesScenario::initializeDefaultParameters();

        auto &fp = floatParams;
        fp[Str::obstaclesMinNumPlatforms] = 2;
        fp[Str::obstaclesMaxNumPlatforms] = 4;

        fp[Str::obstaclesMinLava] = 2;
        fp[Str::obstaclesMaxLava] = 5;

        fp[Str::obstaclesMinHeight] = 1;
        fp[Str::obstaclesMaxHeight] = 3;
    }
};

class ObstaclesHardScenario : public ObstaclesScenario
{
public:
    explicit ObstaclesHardScenario(const std::string &name, Env &env, Env::EnvState &envState)
    : ObstaclesScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesScenario::initializeDefaultParameters();

        auto &fp = floatParams;
        fp[Str::obstaclesMinNumPlatforms] = 2;
        fp[Str::obstaclesMaxNumPlatforms] = 7;

        fp[Str::obstaclesMinGap] = 2;
        fp[Str::obstaclesMaxGap] = 3;

        fp[Str::obstaclesMinLava] = 2;
        fp[Str::obstaclesMaxLava] = 8;

        fp[Str::obstaclesMinHeight] = 2;
        fp[Str::obstaclesMaxHeight] = 4;
    }
};

}
