#pragma once

#include <env/layout_generator.hpp>


class Env
{
public:
    Env();

private:
    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    LayoutGenerator layoutGenerator;
};