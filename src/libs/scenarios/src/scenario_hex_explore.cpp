#include <mazes/honeycombmaze.h>

#include <scenarios/scenario_hex_explore.hpp>

using namespace VoxelWorld;
using namespace Magnum::Math::Literals;


HexExploreScenario::HexExploreScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, maze{*this}
{
}

HexExploreScenario::~HexExploreScenario() = default;

void HexExploreScenario::reset()
{
    maze.minSize = 2, maze.maxSize = 6;
    maze.reset(env, envState);

    auto &hexMaze = maze.getMaze();
    auto &adjList = hexMaze.getAdjacencyList();

    auto mazeScale = maze.getScale();

    for (int attempt = 0; attempt < 10; ++attempt) {
        auto randomCellIdx = randRange(0, adjList.size(), envState.rng);
        auto cellCenter = hexMaze.getCellCenters()[randomCellIdx];

        rewardObjectCoords = Magnum::Vector3{float(cellCenter.first) * mazeScale, 1.5f, float(cellCenter.second) * mazeScale};

        if (Magnum::Vector3{rewardObjectCoords.x(), 0, rewardObjectCoords.z()}.length() < float(maze.getSize()) * maze.getScale()) {
            // if too close to the center of the maze, try again
            continue;
        }
    }
}

void HexExploreScenario::step()
{
    for (auto &agent : env.getAgents()) {
        const auto &t = agent->absoluteTransformation().translation();

        const auto threshold = 0.9f;
        if ((t - rewardObjectCoords).length() < threshold) {
            envState.done = true;
            // TODO: rewards
        }
    }
}

std::vector<Magnum::Vector3> HexExploreScenario::agentStartingPositions()
{
    std::vector<Magnum::Vector3> pos(env.getNumAgents(), {0, 0.2, 0});
    return pos;
}

void HexExploreScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    maze.addDrawablesAndCollisions(drawables, envState);

    // adding reward object
    auto translation = rewardObjectCoords;
    auto &rootObject = envState.scene->addChild<Object3D>();
    auto &bottomHalf = rootObject.addChild<Object3D>();
    bottomHalf.rotateXLocal(180.0_degf).translate({0.0f, -1.0f, 0.0f});
    const auto scale = 2.2f;
    rootObject.scale({0.17f * scale, 0.35f * scale, 0.17f * scale});
    rootObject.translate(translation);

    drawables[DrawableType::Cone].emplace_back(&rootObject, rgb(ColorRgb::VIOLET));
    drawables[DrawableType::Cone].emplace_back(&bottomHalf, rgb(ColorRgb::VIOLET));
}
