#include <random>
#include <util/tiny_logger.hpp>

#include <env/env.hpp>


Env::Env()
{
    TLOG(INFO) << "Creating an environment";

    layoutGenerator.generateFloorWalls(grid);
    layoutGenerator.generateCave(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(numAgents);

    agentStartingPositions = layoutGenerator.agentStartingPositions();
    std::shuffle(agentStartingPositions.begin(), agentStartingPositions.end(), std::mt19937(std::random_device()()));
}

const std::vector<BoundingBox> & Env::getLayoutDrawables()
{
    return layoutDrawables;
}

bool Env::checkStatus(const std::vector<std::unique_ptr<Agent>> &agents)
{
    bool finished = true;

    for (size_t i = 0; i < agents.size(); ++i) {
        const auto t = agents[i]->transformation().translation();

//        TLOG(INFO) << "Exit pad: (" << exitPad.min << ", " << exitPad.max << ")";
//        TLOG(INFO) << "Agent: " << t;
        if (t.x() >= exitPad.min.x() && t.x() <= exitPad.max.x()
            && t.y() >= exitPad.min.y() && t.y() <= exitPad.max.y()
            && t.z() >= exitPad.min.z() && t.z() <= exitPad.max.z()) {
//            TLOG(INFO) << "Agent #" << i << " is on exit pad!";
            continue;
        }

        finished = false;
        break;
    }

    return finished;
}