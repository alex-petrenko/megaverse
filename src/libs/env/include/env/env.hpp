#pragma once

#include <vector>

#include <env/agent.hpp>
#include <env/layout_generator.hpp>


class Env
{
public:
    Env();

    const VoxelGrid<VoxelState> & getVoxelGrid() const
    {
        return grid;
    }

    const std::vector<BoundingBox> & getLayoutDrawables();

    const BoundingBox & getExitPadCoords() const
    {
        return exitPad;
    }

    std::vector<VoxelCoords> getAgentStartingPositions() const
    {
        return std::vector<VoxelCoords>{agentStartingPositions.cbegin(), agentStartingPositions.cbegin() + numAgents};
    }

    bool checkStatus(const std::vector<std::unique_ptr<Agent>> &agents);

public:
    static constexpr int numAgents = 2;

private:
    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    LayoutGenerator layoutGenerator;

    std::vector<BoundingBox> layoutDrawables;

    BoundingBox exitPad;

    std::vector<VoxelCoords> agentStartingPositions;
};