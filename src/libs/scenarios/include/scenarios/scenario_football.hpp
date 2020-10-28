#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_grid_layout.hpp>


namespace VoxelWorld
{

class FootballScenario : public DefaultScenario
{
public:
    explicit FootballScenario(const std::string &name, Env &env, Env::EnvState &envState);

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<VoxelCoords> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return 0; }//TODO

private:
    VoxelGridComponent<VoxelState> vg;
    GridLayoutComponent gridLayoutComponent;

    std::unique_ptr<btSphereShape> collisionShape;
    Object3D *footballObject;
};

}