#pragma once

#include <vector>

#include <util/voxel_grid.hpp>

#include <env/scenario_component.hpp>

#include <scenarios/component_voxel_grid.hpp>


namespace VoxelWorld
{

// TODO: remove. Use room types instead
enum class LayoutType
{
    Empty,
    Cave,
    Walls,
    Pit,
    Towers,
};


class GridLayoutComponent : public ScenarioComponent
{
public:
    class GridLayoutImpl;

public:
    explicit GridLayoutComponent(Scenario &scenario, Rng &rng);

    ~GridLayoutComponent() override;

    void init(int numAgents, LayoutType layoutType = LayoutType::Empty);

    void generate(VoxelGrid<VoxelState> &grid);

    std::vector<BoundingBox> extractPrimitives(VoxelGrid<VoxelState> &grid);

    BoundingBox levelExit(const VoxelGrid<VoxelState> &grid);

    BoundingBox buildingZone(const VoxelGrid<VoxelState> &grid);

    std::vector<VoxelCoords> startingPositions(const VoxelGrid<VoxelState> &grid);

    std::vector<VoxelCoords> objectSpawnPositions(const VoxelGrid<VoxelState> &grid);

    void addLayoutDrawables(DrawablesMap &drawables, Env::EnvState &envState, VoxelGrid<VoxelState> &grid, bool withExit);

    void addBoundingBoxes(DrawablesMap &drawables, Env::EnvState &envState, const Boxes &boxes, int voxelType);

private:
    Rng &rng;

    std::unique_ptr<GridLayoutImpl> generator;

    // TODO decouple layout generation and collision shapes
    std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;
};

}