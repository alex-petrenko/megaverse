#pragma once

#include <scenarios/platforms.hpp>
#include <scenarios/scenario_default.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace Megaverse
{

enum class PlatformType { EMPTY, WALL, LAVA, STEP, GAP };

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
            {Str::obstaclesExtraReward, 0.5f},
            {Str::obstaclesAgentCarriedObjectToExit, 0.0f},
        };
    }

    void initializeDefaultParameters() override
    {
        DefaultScenario::initializeDefaultParameters();

        platformTypes = {PlatformType::WALL, PlatformType::LAVA, PlatformType::STEP, PlatformType::GAP};

        auto &fp = floatParams;
        fp[Str::obstaclesMinNumPlatforms] = 1;
        fp[Str::obstaclesMaxNumPlatforms] = 2;

        fp[Str::obstaclesMinGap] = 1;
        fp[Str::obstaclesMaxGap] = 2;

        fp[Str::obstaclesMinLava] = 1;
        fp[Str::obstaclesMaxLava] = 4;

        fp[Str::obstaclesMinHeight] = 1;
        fp[Str::obstaclesMaxHeight] = 3;

        fp[Str::obstaclesNumAllowedMaxDifficulty] = 1;
    }

    float episodeLengthSec() const override;

    void agentTouchedLava(int agentIdx);

    void agentFell(int agentIdx) override;

protected:
    std::vector<PlatformType> platformTypes;

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

class TestScenario : public ObstaclesScenario
{
public:
    explicit TestScenario(const std::string &name, Env &env, Env::EnvState &envState)
        : ObstaclesScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesScenario::initializeDefaultParameters();

        auto &fp = floatParams;
        fp[Str::obstaclesMinNumPlatforms] = 0;
        fp[Str::obstaclesMaxNumPlatforms] = 0;
        fp[Str::episodeLengthSec] = 6.0f;
    }
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

        fp[Str::obstaclesMinLava] = 3;
        fp[Str::obstaclesMaxLava] = 10;

        fp[Str::obstaclesMinHeight] = 2;
        fp[Str::obstaclesMaxHeight] = 4;
    }
};

class ObstaclesOnePlatformTypeScenario : public ObstaclesScenario
{
public:
    explicit ObstaclesOnePlatformTypeScenario(const std::string &name, Env &env, Env::EnvState &envState)
        : ObstaclesScenario(name, env, envState)
    {
    }

    RewardShaping defaultRewardShaping() const override
    {
        auto shaping = ObstaclesScenario::defaultRewardShaping();
        shaping[Str::obstaclesAgentCarriedObjectToExit] = 1.0f;
        return shaping;
    }

    void initializeDefaultParameters() override
    {
        ObstaclesScenario::initializeDefaultParameters();

        auto &fp = floatParams;
        fp[Str::obstaclesMinNumPlatforms] = 1;
        fp[Str::obstaclesMaxNumPlatforms] = 4;

        fp[Str::obstaclesMinGap] = 1;
        fp[Str::obstaclesMaxGap] = 3;
        fp[Str::obstaclesMinLava] = 2;
        fp[Str::obstaclesMaxLava] = 10;
        fp[Str::obstaclesMinHeight] = 1;
        fp[Str::obstaclesMaxHeight] = 3;

        fp[Str::obstaclesNumAllowedMaxDifficulty] = 1;
    }
};

class ObstaclesOnlyWallsScenario : public ObstaclesOnePlatformTypeScenario
{
public:
    explicit ObstaclesOnlyWallsScenario(const std::string &name, Env &env, Env::EnvState &envState)
        : ObstaclesOnePlatformTypeScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesOnePlatformTypeScenario::initializeDefaultParameters();
        platformTypes = {PlatformType::WALL};
    }
};

class ObstaclesOnlyStepsScenario : public ObstaclesOnePlatformTypeScenario
{
public:
    explicit ObstaclesOnlyStepsScenario(const std::string &name, Env &env, Env::EnvState &envState)
        : ObstaclesOnePlatformTypeScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesOnePlatformTypeScenario::initializeDefaultParameters();
        platformTypes = {PlatformType::STEP};
    }
};

class ObstaclesOnlyLavaScenario : public ObstaclesOnePlatformTypeScenario
{
public:
    explicit ObstaclesOnlyLavaScenario(const std::string &name, Env &env, Env::EnvState &envState)
        : ObstaclesOnePlatformTypeScenario(name, env, envState)
    {
    }

    void initializeDefaultParameters() override
    {
        ObstaclesOnePlatformTypeScenario::initializeDefaultParameters();
        platformTypes = {PlatformType::LAVA};
    }
};

}
