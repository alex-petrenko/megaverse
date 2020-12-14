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

class ObstaclesScenario : public DefaultScenario, public ObjectStackingCallbacks, public FallDetectionCallbacks
{
public:
    explicit ObstaclesScenario(const std::string &name, Env &env, Env::EnvState &envState);

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override { return agentSpawnPositions; }

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return 0; }//TODO

    float episodeLengthSec() const override;

    void agentTouchedLava(int agentIdx);

    void agentFell(int agentIdx) override;

private:
    VoxelGridComponent<VoxelWithPhysicsObjects> vg;
    PlatformsComponent platformsComponent;
    ObjectStackingComponent<VoxelWithPhysicsObjects> objectStackingComponent;
    FallDetectionComponent<VoxelWithPhysicsObjects> fallDetection;

    std::vector<VoxelCoords> objectSpawnPositions;
    std::vector<Magnum::Vector3> agentSpawnPositions;

    int numPlatforms{};
};

}
