#pragma once

#include <env/physics.hpp>


namespace VoxelWorld
{

struct VoxelState
{
    explicit VoxelState() = default;

    explicit VoxelState(bool solid)
        : solid{solid}
    {
    }

    bool solid = false;

    // TODO: more flexible mechanism?
    RigidBody *obj = nullptr;
};

}