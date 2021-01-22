#pragma once

#include <env/const.hpp>
#include <env/physics.hpp>


namespace VoxelWorld
{

enum VoxelType
{
    VOXEL_EMPTY = 0,        // is not a part of the layout, but may contain an object, so there's still entry in the voxel grid
    VOXEL_SOLID = 0b1,        // whether a voxel requires collision or not, i.e. a regular layout
    VOXEL_OPAQUE = 0b10,  // whether a voxel needs to be drawn on screen, don't set this if you want an invisible wall
};

struct VoxelState
{
    VoxelState()
    : voxelType{VOXEL_EMPTY}
    , terrain{0}
    {
    }

    bool solid() const { return voxelType & VOXEL_SOLID; }
    bool empty() const { return !solid(); }
    bool opaque() const { return voxelType & VOXEL_OPAQUE; }

    static uint8_t generateType(bool solid, bool opaque)
    {
        return solid | (opaque << 1);
    }

public:
    uint8_t voxelType{}, terrain{};
    ColorRgb color{ColorRgb::LAYOUT_DEFAULT};
};

template<typename VoxelT>
auto makeVoxel(int type, int terrain = 0, ColorRgb color = ColorRgb::LAYOUT_DEFAULT)
{
    VoxelT v;
    v.voxelType = type, v.terrain = terrain, v.color = color;
    return v;
}

}