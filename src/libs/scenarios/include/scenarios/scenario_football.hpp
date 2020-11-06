#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/grid_layout_utils.hpp>


namespace VoxelWorld
{

class FootballScenario : public DefaultScenario
{
private:
    class FootballLayout;

public:
    explicit FootballScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~FootballScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<VoxelCoords> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return 0; }//TODO

private:
    VoxelGridComponent<VoxelState> vg;
    PlatformsComponent platformsComponent;

    std::unique_ptr<btSphereShape> collisionShape;
    Object3D *footballObject = nullptr;

    std::unique_ptr<FootballLayout> layout;
};

}