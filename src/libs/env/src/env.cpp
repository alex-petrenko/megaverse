#include <util/tiny_logger.hpp>

#include <env/env.hpp>


Env::Env()
{
    TLOG(INFO) << "Creating an environment";

    layoutGenerator.generateFloorWalls(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(numAgents);
}

const std::vector<BoundingBox> & Env::getLayoutDrawables()
{
    return layoutDrawables;
}
