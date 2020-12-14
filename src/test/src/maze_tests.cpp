#include <gtest/gtest.h>

#include <util/tiny_logger.hpp>

#include <mazes/kruskal.h>
#include <mazes/honeycombmaze.h>


using namespace VoxelWorld;


TEST(maze, honeycomb)
{
    const auto size = 5;
    auto *maze = new HoneyCombMaze{size};
    auto *algorithm = new Kruskal;

    TLOG(DEBUG) << "Initialising graph...";
    maze->InitialiseGraph();
    TLOG(DEBUG) << "Generating maze...";
    maze->GenerateMaze(algorithm);

    const auto outputprefix = "/tmp/maze";
    TLOG(DEBUG) << "Rendering maze to '" << outputprefix << ".svg'...";
    maze->PrintMazeSVG(outputprefix);

    auto adjList = maze->getAdjacencyList();
    const auto [xmin, ymin, xmax, ymax] = maze->GetCoordinateBounds();

    TLOG(DEBUG) << system("eog /tmp/maze.svg");
}
