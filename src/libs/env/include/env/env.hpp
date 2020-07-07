#pragma once

#include <vector>

#include <env/agent.hpp>
#include <env/layout_generator.hpp>


class Env
{
public:
    Env();

    const std::vector<BoundingBox> & getLayoutDrawables();

    const BoundingBox & getExitPadCoords() const
    {
        return exitPad;
    }

private:
    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    LayoutGenerator layoutGenerator;

    std::vector<BoundingBox> layoutDrawables;

    BoundingBox exitPad;

    static constexpr int numAgents = 1;
    std::vector<Agent> agents;
};