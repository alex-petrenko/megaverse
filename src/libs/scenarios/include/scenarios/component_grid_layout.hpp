#pragma once

#include <vector>

#include <util/voxel_grid.hpp>

#include <env/scenario_component.hpp>

#include <scenarios/component_voxel_grid.hpp>


namespace VoxelWorld
{

class GridLayoutComponent : public ScenarioComponent
{
public:
    explicit GridLayoutComponent(Scenario &scenario)
    : ScenarioComponent{scenario}
    {
    }

    void addBoundingBoxes(DrawablesMap &drawables, Env::EnvState &envState, const Boxes &boxes, int voxelType);

    void addTerrain(DrawablesMap &drawables, Env::EnvState &envState, TerrainType type, const BoundingBox &bb);

    void addInteractiveObjects(DrawablesMap &drawables, Env::EnvState &envState, const std::vector<VoxelCoords> &objectPositions, VoxelGrid<VoxelState> &grid);

private:
    std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;
};

}