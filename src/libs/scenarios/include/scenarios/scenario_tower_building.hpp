#pragma once

#include <env/scenario.hpp>

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

class TowerBuildingScenario : public DefaultScenario, public ObjectStackingCallbacks
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
    explicit TowerBuildingScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~TowerBuildingScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int) const override { return float(highestTower); }

    RewardShaping defaultRewardShaping() const override
    {
        return {
            {Str::teamSpirit, 0.1f},  // replaces the default value
            {Str::towerPickedUpObject, 0.1f},
            {Str::towerVisitedBuildingZoneWithObject, 0.1f},
            {Str::towerBuildingReward, 1.0f},
        };
    }

    float episodeLengthSec() const override;

    // Callbacks
    bool canPlaceObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;
    void placedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;
    void pickedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) override;

private:
    bool isInBuildingZone(const VoxelCoords &c) const;
    float calculateTowerReward() const;

    static float buildingRewardCoeffForHeight(float height);
    void addCollectiveReward(int agentIdx);

private:
    VoxelGridComponent<VoxelWithPhysicsObjects> vg;
    ObjectStackingComponent<VoxelWithPhysicsObjects> objectStackingComponent;
    PlatformsComponent platformsComponent;

    int highestTower = 0;
    BoundingBox buildingZone;
    std::unordered_set<VoxelCoords> objectsInBuildingZone;
    float currBuildingZoneReward = 0.0f;

    std::vector<AgentState> agentState;

    // previous reward associated with an object (so we can subtract it when we move the object elsewhere)
    std::map<Object3D *, float> previousReward;

    std::unique_ptr<TowerBuildingPlatform> platform;
};

}
