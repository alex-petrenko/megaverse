#pragma once

#include <env/physics.hpp>


namespace VoxelWorld
{

enum VoxelType
{
    VOXEL_EMPTY = 0,        // is not a part of the layout, but may contain an object, so there's still entry in the voxel grid
    VOXEL_SOLID = 1,        // whether a voxel requires collision or not, i.e. a regular layout
    VOXEL_OPAQUE = 1 << 1,  // whether a voxel needs to be drawn on screen, don't set this if you want an invisible wall
};

struct VoxelState
{
    VoxelState()
    : voxelType{VOXEL_EMPTY}
    , terrain{0}
    {
    }

    bool solid() const { return voxelType & VOXEL_SOLID; }
    bool opaque() const { return voxelType & VOXEL_OPAQUE; }

public:
    uint8_t voxelType{}, terrain{};
};

template<typename VoxelT>
auto makeVoxel(int type, int terrain = 0)
{
    VoxelT v;
    v.voxelType = type, v.terrain = terrain;
    return v;
}

}