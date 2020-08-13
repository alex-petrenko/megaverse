#pragma once

#include <util/util.hpp>
#include <util/voxel_grid.hpp>

#include <env/voxel_state.hpp>


enum class LayoutType
{
    Empty,
    Cave,
    Walls,
    Pit,
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


class LayoutGenerator
{
public:
    class LayoutGeneratorImpl;

public:
    explicit LayoutGenerator(Rng &rng);
    ~LayoutGenerator();

    void init(int numAgents, LayoutType layoutType = LayoutType::Empty);

    void generate(VoxelGrid<VoxelState> &grid);

    std::vector<BoundingBox> extractPrimitives(VoxelGrid<VoxelState> &grid);

    BoundingBox levelExit(const VoxelGrid<VoxelState> &grid);

    std::vector<VoxelCoords> startingPositions(const VoxelGrid<VoxelState> &grid);

    std::vector<VoxelCoords> objectSpawnPositions(const VoxelGrid<VoxelState> &grid);

private:
    Rng &rng;

    std::unique_ptr<LayoutGeneratorImpl> generator;
};
