#pragma once

#include <util/voxel_grid.hpp>

#include <env/scenario_component.hpp>


namespace VoxelWorld
{

enum class LayoutType
{
    Empty,
    Cave,
    Walls,
    Pit,
    Towers,
};


struct BoundingBox
{
    VoxelCoords min, max;

    void addPoint(VoxelCoords v)
    {
        if (v.x() <= min.x() && v.y() <= min.y() && v.z() <= min.z())
            min = v;
        else if (v.x() >= max.x() && v.y() >= max.y() && v.z() >= max.z())
            max = v;
    }
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

    void addLayoutDrawables(DrawablesMap &drawables, Env::EnvState &envState, VoxelGrid<VoxelState> &grid);

private:
    Rng &rng;

    std::unique_ptr<GridLayoutImpl> generator;

    // TODO decouple layout generation and collision shapes
    std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;
};

}