#pragma once


#include <env/scenario.hpp>

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/grid_layout_utils.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

class TowerBuilding : public DefaultScenario, public ObjectStackingCallbacks
{
private:
    class TowerBuildingPlatform;

public:
    struct AgentState
    {
        bool pickedUpObject = false;
        bool visitedBuildingZoneWithObject = false;
    };

public:
    explicit TowerBuilding(const std::string &name, Env &env, Env::EnvState &envState);

    ~TowerBuilding() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return float(highestTower); }

    // Callbacks
    bool canPlaceObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;
    void placedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;
    void pickedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;

private:
    bool isInBuildingZone(const VoxelCoords &c) const;

    float buildingReward(float height) const;
    void addCollectiveReward(int agentIdx, float rewardDelta) const;

private:
    VoxelGridComponent<VoxelWithPhysicsObjects> vg;
    ObjectStackingComponent<VoxelWithPhysicsObjects> objectStackingComponent;
    PlatformsComponent platformsComponent;

    int highestTower = 0;
    BoundingBox buildingZone;

    std::vector<AgentState> agentState;

    // previous reward associated with an object (so we can subtract it when we move the object elsewhere)
    std::map<Object3D *, float> previousReward;

    std::unique_ptr<TowerBuildingPlatform> platform;
};

}
