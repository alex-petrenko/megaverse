#pragma once

#include <scenarios/platforms.hpp>
#include <scenarios/scenario_default.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_grid_layout.hpp>


namespace VoxelWorld
{

class ObstaclesScenario : public DefaultScenario
{
public:
    explicit ObstaclesScenario(const std::string &name, Env &env, Env::EnvState &envState);

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<VoxelCoords> agentStartingPositions() override { return agentSpawnPositions; }

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return 0; }//TODO

    float episodeLengthSec() const override;

private:
    VoxelGridComponent<VoxelState> vg;
    GridLayoutComponent gridLayout;
    PlatformsComponent platformsComponent;

    std::vector<VoxelCoords> agentSpawnPositions, objectSpawnPositions;

    int numPlatforms{};
};

}