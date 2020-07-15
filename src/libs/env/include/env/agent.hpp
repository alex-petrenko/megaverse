#pragma once

#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/magnum.hpp>
#include <util/voxel_grid.hpp>

#include <env/voxel_state.hpp>


class Agent : public Object3D
{
public:
    explicit Agent(Object3D *parent);

public:
    std::unique_ptr<Object3D> cameraObject;
    std::unique_ptr<Magnum::SceneGraph::Camera3D> camera;
};


void moveAgent(Agent &a, const Magnum::Vector3 &delta, const VoxelGrid<VoxelState> &vg, int depth=0);