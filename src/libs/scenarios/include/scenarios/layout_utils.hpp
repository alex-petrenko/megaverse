#pragma once

#include <util/voxel_grid.hpp>

#include <env/env.hpp>

#include <scenarios/component_voxel_grid.hpp>


namespace VoxelWorld
{
    void addBoundingBoxes(DrawablesMap &drawables, Env::EnvState &envState, const Boxes &boxes, int voxelType, float voxelSize = 1.0f);
    void addTerrain(DrawablesMap &drawables, Env::EnvState &envState, TerrainType type, const BoundingBox &bb, float voxelSize = 1.0f);

    void addStaticCollidingBox(
        DrawablesMap &drawables, Env::EnvState &envState,
        Magnum::Vector3 scale, Magnum::Vector3 translation, ColorRgb color
    );

    Object3D * addCylinder(DrawablesMap &drawables, Object3D &parent, Magnum::Vector3 translation, Magnum::Vector3 scale, ColorRgb color);
    Object3D * addSphere(DrawablesMap &drawables, Object3D &parent, Magnum::Vector3 translation, Magnum::Vector3 scale, ColorRgb color);
    Object3D * addPillar(DrawablesMap &drawables, Object3D &parent, Magnum::Vector3 translation, Magnum::Vector3 scale, ColorRgb color);
    Object3D * addDiamond(DrawablesMap &drawables, Object3D &parent, Magnum::Vector3 translation, Magnum::Vector3 scale, ColorRgb color);
}