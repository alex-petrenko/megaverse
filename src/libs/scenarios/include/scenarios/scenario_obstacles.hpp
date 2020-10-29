#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_grid_layout.hpp>


namespace VoxelWorld
{

enum TerrainType
{
    TERRAIN_EXIT = 1,
    TERRAIN_LAVA = 1 << 1,
};

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

private:
    void addTerrain(DrawablesMap &drawables, TerrainType type, const BoundingBox &bb);

    static ColorRgb terrainColor(TerrainType type) ;

private:
    VoxelGridComponent<VoxelState> vg;
    GridLayoutComponent gridLayout;

    std::vector<VoxelCoords> agentSpawnPositions, objectSpawnPositions;
    std::map<TerrainType, std::vector<BoundingBox>> terrain;

    std::unique_ptr<Object3D> levelRoot;
};

}