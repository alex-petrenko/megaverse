#pragma once

#include <deque>

#include <scenarios/scenario_default.hpp>
#include <scenarios/grid_layout_utils.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>


namespace VoxelWorld
{

struct VoxelBoxAGone : public VoxelState
{
    RigidBody *disappearingPlatform = nullptr;
};

class BoxAGoneScenario : public DefaultScenario, public FallDetectionCallbacks
{
private:
    class BoxAGonePlatform;

    struct AgentState
    {
        RigidBody *lastPlatform = nullptr;
    };

    struct PlatformState
    {
        int remainingTicks = 0;
        VoxelCoords coords;
        RigidBody *temporaryPlatform = nullptr;
    };

public:
    explicit BoxAGoneScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~BoxAGoneScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    void addDisappearingPlatforms(DrawablesMap &drawables);

    float trueObjective() const override { return 0; } // TODO

    void initializeDefaultParameters() override
    {
        auto &fp = floatParams;
        fp[Str::episodeLengthSec] = 600.0f;
        fp[Str::verticalLookLimitRad] = 0.75f;
    }

private:
    constexpr static float voxelSize = 2.0f;
    constexpr static int platformSize = 24;

    VoxelGridComponent<VoxelBoxAGone> vg;
    PlatformsComponent platformsComponent;

    FallDetectionComponent<VoxelBoxAGone> fallDetection;

    std::unique_ptr<BoxAGonePlatform> platform;

    std::vector<std::pair<VoxelCoords, ColorRgb>> disappearingPlatforms;
    std::vector<VoxelCoords> spawnPositions;

    std::vector<AgentState> agentStates;
    std::map<RigidBody *, PlatformState> platformStates;
    std::deque<RigidBody *> extraPlatforms;
};

}