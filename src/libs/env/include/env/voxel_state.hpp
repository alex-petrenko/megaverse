#pragma once

#include <env/physics.hpp>


struct VoxelState
{
    explicit VoxelState() = default;

    explicit VoxelState(bool solid)
    : solid{solid}
    {
    }

    bool solid = false;
    RigidBody *obj = nullptr;
};