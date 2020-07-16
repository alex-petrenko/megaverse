#pragma once

#include <memory>

#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/magnum.hpp>
#include <util/voxel_grid.hpp>

#include <env/voxel_state.hpp>


class Agent : public Object3D
{
public:
    explicit Agent(Object3D *parent);
    ~Agent() override;

    void move(const Magnum::Vector3 &delta, const VoxelGrid<VoxelState> &vg, int depth=0);

private:
    void ensureAgentNotOnVoxelBoundary(const VoxelCoords &v);

public:
    bool allowLookUp = false;

    Magnum::SceneGraph::Camera3D *camera;
};


