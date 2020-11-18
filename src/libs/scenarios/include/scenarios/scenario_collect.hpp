#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/grid_layout_utils.hpp>
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

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return cumulativeReward; }

private:
    VoxelGridComponent<VoxelCollect> vg;
    ObjectStackingComponent<VoxelCollect> objectStackingComponent;
    FallDetectionComponent<VoxelCollect> fallDetection;

    std::vector<VoxelCoords> objectPositions, rewardPositions;
    std::vector<Magnum::Vector3> agentPositions;

    int numPositiveRewards = 0, positiveRewardsCollected = 0;
    float cumulativeReward = 0;
};

}