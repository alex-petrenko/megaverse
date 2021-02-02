#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

struct VoxelCollect : public VoxelWithPhysicsObjects
{
    Object3D *rewardObject = nullptr;
    int reward = 0;
};


class CollectScenario : public DefaultScenario, public ObjectStackingCallbacks, public FallDetectionCallbacks
{
public:
    explicit CollectScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~CollectScenario() override;

    // Scenario interface
    void reset() override;

    void createLandscape();

    void step() override;

    void agentFell(int agentIdx) override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int /*agentIdx*/) const override { return solved; }

    RewardShaping defaultRewardShaping() const override
    {
        return {
            {Str::collectSingleGood, 1.0f},
            {Str::collectSingleBad, -1.0f},
            {Str::collectAll, 5.0f},
            {Str::collectAbyss, -0.5f},
        };
    }

    float episodeLengthSec() const override
    {
        // add a little bit of time for every extra reward object
        return Scenario::episodeLengthSec() + 2.0f * rewardPositions.size();
    }

private:
    bool solved = false;

    VoxelGridComponent<VoxelCollect> vg;
    ObjectStackingComponent<VoxelCollect> objectStackingComponent;
    FallDetectionComponent<VoxelCollect> fallDetection;

    std::vector<VoxelCoords> objectPositions, rewardPositions;
    std::vector<Magnum::Vector3> agentPositions;

    int numPositiveRewards = 0, positiveRewardsCollected = 0;
};

}