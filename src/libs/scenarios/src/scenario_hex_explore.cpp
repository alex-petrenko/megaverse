#include <mazes/honeycombmaze.h>

#include <scenarios/const.hpp>
#include <scenarios/layout_utils.hpp>
#include <scenarios/scenario_hex_explore.hpp>

using namespace VoxelWorld;

using namespace Magnum;
using namespace Magnum::Math::Literals;


HexExploreScenario::HexExploreScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, maze{*this}
{
}

HexExploreScenario::~HexExploreScenario() = default;

void HexExploreScenario::reset()
{
    solved = false;

    maze.minSize = 2, maze.maxSize = 7;
    maze.omitWallsProbabilityMin = 0.1f, maze.omitWallsProbabilityMax = 0.4f;
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

    rewardObject = nullptr;
}

void HexExploreScenario::step()
{
    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto &agent = env.getAgents()[i];
        const auto t = agent->absoluteTransformation().translation();

        const auto threshold = 0.9f;
        if ((t - rewardObjectCoords).length() < threshold && !solved) {
            solved = true;
            doneWithTimer();
            rewardTeam(Str::exploreSolved, i, 1);
            rewardObject->translate({1e3, 1e3, 1e3});
            break;
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
    const auto scale = 1.9f;
    rewardObject = addDiamond(drawables, *envState.scene, rewardObjectCoords - Vector3{0, 0.3, 0}, {0.17f * scale, 0.35f * scale, 0.17f * scale}, ColorRgb::VIOLET);
}
