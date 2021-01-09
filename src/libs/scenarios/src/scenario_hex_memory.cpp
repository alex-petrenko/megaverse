#include <mazes/honeycombmaze.h>

#include <scenarios/layout_utils.hpp>
#include <scenarios/scenario_hex_memory.hpp>

using namespace VoxelWorld;


HexMemoryScenario::HexMemoryScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, maze{*this}
{

}

HexMemoryScenario::~HexMemoryScenario() = default;

void HexMemoryScenario::reset()
{
    goodObjects.clear(), badObjects.clear();

    maze.minSize = 2, maze.maxSize = 10;
    maze.reset(env, envState);

    auto &hexMaze = maze.getMaze();
    auto &adjList = hexMaze.getAdjacencyList();

    auto mazeScale = maze.getScale();

    float minDistanceToCenter = 1e9f;
    int centerCellIdx = 0;

    // hacky way to find the cell closest to center
    for (int cellIdx = 0; cellIdx < int(adjList.size()); ++cellIdx) {
        auto cellCenter = hexMaze.getCellCenters()[cellIdx];
        auto distanceToCenter = sqrt(sqr(cellCenter.first) + sqr(cellCenter.second));

        if (distanceToCenter < minDistanceToCenter) {
            centerCellIdx = cellIdx;
            minDistanceToCenter = distanceToCenter;
        }
    }

    auto mazeCenter = hexMaze.getCellCenters()[centerCellIdx];
    landmarkLocation = Magnum::Vector3(mazeCenter.first * mazeScale, 1.0f, mazeCenter.second * mazeScale);

    std::vector<Magnum::Vector3> objectCoordinates;

    for (int cellIdx = 0; cellIdx < int(adjList.size()); ++cellIdx) {
        if (cellIdx == centerCellIdx)
            continue;

        auto cellCenter = hexMaze.getCellCenters()[cellIdx];
        auto coord = Magnum::Vector3(cellCenter.first, 0.5f, cellCenter.second);
        auto offset = Magnum::Vector3(frand(envState.rng) - 0.5f, 0, frand(envState.rng) - 0.5f);
        coord += offset;
        objectCoordinates.emplace_back(coord.x() * mazeScale, coord.y(), coord.z() * mazeScale);
    }

    std::shuffle(objectCoordinates.begin(), objectCoordinates.end(), envState.rng);

    float cellWithObjectsFraction = frand(envState.rng) * 0.1f + 0.1f;
    long numCellsWithGoodObjects = std::lround(ceilf(cellWithObjectsFraction * objectCoordinates.size()));
    long numCellsWithBadObjects = std::lround(ceilf(cellWithObjectsFraction * objectCoordinates.size()));

    goodObjects = std::vector<Magnum::Vector3>(objectCoordinates.begin(), objectCoordinates.begin() + numCellsWithGoodObjects);
    if (int(objectCoordinates.size()) >= numCellsWithGoodObjects + numCellsWithBadObjects)
        badObjects = std::vector<Magnum::Vector3>(objectCoordinates.begin() + numCellsWithGoodObjects, objectCoordinates.begin() + numCellsWithGoodObjects + numCellsWithBadObjects);
}

void HexMemoryScenario::step()
{
    Scenario::step();
}

std::vector<Magnum::Vector3> HexMemoryScenario::agentStartingPositions()
{
    const auto rotationBetweenAgents = float(2 * M_PI / env.getNumAgents());
    std::vector<Magnum::Vector3> pos(env.getNumAgents());

    for (int i = 0; i < env.getNumAgents(); ++i)
        pos[i] = 1.5f * Magnum::Vector3{sinf(rotationBetweenAgents * float(i)), 0.3, cosf(rotationBetweenAgents * float(i))};

    return pos;
}

void HexMemoryScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    maze.addDrawablesAndCollisions(drawables, envState);

    // adding landmark object
    Magnum::Vector3 landmarkScale {0.5, 2, 0.5}, objectScale = landmarkScale * 0.5f;
    addPillar(drawables, *envState.scene, landmarkLocation, landmarkScale, ColorRgb::VIOLET);


    for (const auto &c : goodObjects)
        addPillar(drawables, *envState.scene, c, objectScale, ColorRgb::VIOLET);

    for (const auto &c : badObjects)
        addPillar(drawables, *envState.scene, c, objectScale, ColorRgb::RED);
}
