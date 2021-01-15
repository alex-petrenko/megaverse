#pragma once

#include <map>
#include <tuple>

#include <env/env.hpp>

#include <Magnum/Trade/MeshData.h>


namespace VoxelWorld
{

void initPrimitives(std::map<DrawableType, Magnum::Trade::MeshData> &meshData);

inline std::tuple<float, float, float> cameraParameters()
{
    float fov = 100, near = 0.001, far = 150.0;
    return std::make_tuple(fov, near, far);
}

}