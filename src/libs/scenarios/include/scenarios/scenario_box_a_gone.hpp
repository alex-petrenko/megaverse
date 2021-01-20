#pragma once

#include <deque>

#include <scenarios/scenario_default.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>


namespace VoxelWorld
{

struct VoxelBoxAGone : public VoxelState
{
    RigidBody *disappearingPlatform = nullptr;
} __attribute__((aligned(8)));

class BoxAGoneScenario : public DefaultScenario, public FallDetectionCallbacks
{
private:
    class BoxAGonePlatform;

    struct AgentState
    {
        RigidBody *lastPlatform = nullptr;
        float secondsBeforeTouchedFloor = 0.0f;
    };

    struct PlatformState
    {
        int remainingTicks = 0;
        VoxelCoords coords;
        RigidBody *temporaryPlatform = nullptr;
    } __attribute__((aligned(32)));

public:
    explicit BoxAGoneScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~BoxAGoneScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    void addDisappearingPlatforms(DrawablesMap &drawables);

    /**
     * Different trueObjective depending on whether this is a single-agent or competitive setting.
     */
    float trueObjective(int agentIdx) const override
    {
        if (env.getNumAgents() > 1) {
            // last man standing wins
            float maxSecondsBeforeTouchingFloor = 0.0f;
            int bestAgent = 0;
            for (int i = 0; i < env.getNumAgents(); ++i)
                if (agentStates[i].secondsBeforeTouchedFloor > maxSecondsBeforeTouchingFloor)
                    bestAgent = i, maxSecondsBeforeTouchingFloor = agentStates[i].secondsBeforeTouchedFloor;

            return agentIdx == bestAgent;  // winner gets 1, other agents get 0
        } else {
            // fraction of the episode for which we managed to avoid the floor
            return agentStates[agentIdx].secondsBeforeTouchedFloor / episodeLengthSec();
        }
    }

    void initializeDefaultParameters() override
    {
        auto &fp = floatParams;
        fp[Str::episodeLengthSec] = 300.0f;
        fp[Str::verticalLookLimitRad] = 0.75f;
    }

    RewardShaping defaultRewardShaping() const override
    {
        return {
            {Str::boxagoneTouchedFloor,  -0.1f},
            {Str::boxagonePerStepReward, 0.01f},
        };
    }

    /**
     * Each agent is for himself.
     */
    int teamAffinity(int agentIdx) const override { return agentIdx; }

private:
    constexpr static float voxelSize = 2.0f;
    constexpr static int platformSize = 24;

    bool finished = false;

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