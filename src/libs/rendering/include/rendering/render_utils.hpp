#pragma once

#include <map>
#include <tuple>

#include <env/env.hpp>

#include <Magnum/Trade/MeshData.h>


namespace VoxelWorld
{

void initPrimitives(std::map<DrawableType, Magnum::Trade::MeshData> &meshData);

class Overview
{
public:
    void saveTransformation()
    {
        rootTransformation = root->transformation();
        verticalTiltTransformation = verticalTilt->transformation();
    }

    void restoreTransformation() const
    {
        root->setTransformation(rootTransformation);
        verticalTilt->setTransformation(verticalTiltTransformation);
    }

    void reset(Object3D *parent);

public:
    Object3D *root{}, *verticalTilt{};

    Magnum::SceneGraph::Camera3D *camera{};
    bool enabled = false;

    float verticalRotation = 0.0f;

    Magnum::Matrix4 rootTransformation{}, verticalTiltTransformation{};
};

}