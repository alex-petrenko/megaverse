#pragma once

#include <util/voxel_grid.hpp>

#include <env/voxel_state.hpp>


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
    void generateFloor(VoxelGrid<VoxelState> &grid);
    void generateFloorWalls(VoxelGrid<VoxelState> &grid);

    std::vector<BoundingBox> extractPrimitives(VoxelGrid<VoxelState> &grid);

private:
    static constexpr int width = 150, length = 100, height = 30;
};
