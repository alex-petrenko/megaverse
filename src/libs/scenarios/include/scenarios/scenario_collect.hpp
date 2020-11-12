#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/grid_layout_utils.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

struct VoxelCollect : public VoxelWithPhysicsObjects
{
    Object3D *rewardObject = nullptr;
    int reward = 0;
};


class CollectScenario : public DefaultScenario, public ObjectStackingCallbacks
{
private:
    class CollectLayout;

public:
    explicit CollectScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~CollectScenario() override;

    // Scenario interface
    void reset() override;

    void createLandscape();

    void step() override;

    std::vector<VoxelCoords> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return cumulativeReward; }

private:
    VoxelGridComponent<VoxelCollect> vg;
    ObjectStackingComponent<VoxelCollect> objectStackingComponent;

    std::vector<VoxelCoords> agentPositions, objectPositions, rewardPositions;

    int numPositiveRewards = 0, positiveRewardsCollected = 0;
    float cumulativeReward = 0;
};

}