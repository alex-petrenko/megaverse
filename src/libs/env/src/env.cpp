#include <util/tiny_logger.hpp>

#include <env/env.hpp>


Env::Env()
{
    TLOG(INFO) << "Creating an environment";

    layoutGenerator.generateFloorWalls(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);
}

const std::vector<BoundingBox> & Env::getLayoutDrawables()
{
    return layoutDrawables;
}
