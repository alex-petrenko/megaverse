#pragma once

#include <vector>

#include <env/layout_generator.hpp>


class Env
{
public:
    Env();

    const std::vector<BoundingBox> & getLayoutDrawables();

private:
    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    LayoutGenerator layoutGenerator;

    std::vector<BoundingBox> layoutDrawables;
};