#include <util/tiny_logger.hpp>

#include <env/env.hpp>


Env::Env()
{
    TLOG(INFO) << "Creating an environment";

    layoutGenerator.generateFloorWalls(grid);
    auto drawables = layoutGenerator.extractPrimitives(grid);
}
