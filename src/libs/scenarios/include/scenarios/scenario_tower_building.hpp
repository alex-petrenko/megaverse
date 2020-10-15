#pragma once


#include <env/scenario.hpp>

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_grid_layout.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

class TowerBuilding : public DefaultScenario, public ObjectStackingCallbacks
{
public:
    struct AgentState
    {
        bool pickedUpObject = false;
        bool visitedBuildingZoneWithObject = false;
    };

public:
    explicit TowerBuilding(const std::string &name, Env &env, Env::EnvState &envState);

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<VoxelCoords> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return highestTower; }

    // Callbacks
    bool canPlaceObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;
    void placedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;
    void pickedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;

private:
    bool isInBuildingZone(const VoxelCoords &c) const;

    float buildingReward(float height) const;
    void addCollectiveReward(int agentIdx, float rewardDelta) const;

private:
    VoxelGridComponent<VoxelState> vg;
    ObjectStackingComponent<VoxelState> objectStackingComponent;
    GridLayoutComponent gridLayoutComponent;

    int highestTower = 0;
    BoundingBox buildingZone;

    std::vector<AgentState> agentState;

    // previous reward associated with an object (so we can subtract it when we move the object elsewhere)
    std::map<Object3D *, float> previousReward;
};

}
