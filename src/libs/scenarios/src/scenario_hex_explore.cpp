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

    maze.minSize = 2, maze.maxSize = 8;
    maze.omitWallsProbabilityMin = 0.1f, maze.omitWallsProbabilityMax = 0.4f;
    maze.reset(env, envState);

    auto &hexMaze = maze.getMaze();
    auto &adjList = hexMaze.getAdjacencyList();

    auto mazeScale = maze.getScale();

    auto randomCellIdx = randRange(0, adjList.size(), envState.rng);
    auto cellCenter = hexMaze.getCellCenters()[randomCellIdx];

    rewardObjectCoords = Magnum::Vector3{float(cellCenter.first) * mazeScale, 0, float(cellCenter.second) * mazeScale};

    rewardObject = nullptr;
}

void HexExploreScenario::step()
{
    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto &agent = env.getAgents()[i];
        const auto t = agent->absoluteTransformation().translation();

        const auto threshold = 1.2;
        const auto distance = (t - rewardObjectCoords).length();
        if (distance < threshold && !solved) {
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
    std::vector<Magnum::Vector3> positions;

    auto &hexMaze = maze.getMaze();
    auto mazeScale = maze.getScale();
    std::vector<int> cellIndices(hexMaze.getAdjacencyList().size(), 0);
    std::iota(cellIndices.begin(), cellIndices.end(), 0);
    std::shuffle(cellIndices.begin(), cellIndices.end(), envState.rng);

    float furtherstDistance = 0;

    for (auto cellIdx : cellIndices) {
        auto cellCenter = hexMaze.getCellCenters()[cellIdx];

        auto spawnPos = Vector3{float(cellCenter.first) * mazeScale, 0.1, float(cellCenter.second) * mazeScale};
        auto distance = (rewardObjectCoords - spawnPos).length();
        auto rotation = float(2 * M_PI / env.getNumAgents());

        if (distance > furtherstDistance) {
            positions.clear();
            for (int i = 0; i < env.getNumAgents(); ++i) {
                auto delta = Vector3{sinf(float(i) * rotation), 0, cosf(float(i) * rotation)};
                positions.emplace_back(spawnPos + delta);
            }

            furtherstDistance = distance;
        }

        if (distance > float(maze.getSize()) * maze.getScale())
            break;
    }

    if (positions.empty()) {
        TLOG(DEBUG) << "Could not generate valid spawn positions for the agents";
        positions = std::vector<Vector3>(env.getNumAgents(), Vector3{0, 1, 0});
    }

    return positions;
}

void HexExploreScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    maze.addDrawablesAndCollisions(drawables, envState);

    // adding reward object
    const auto scale = 1.9f;
    rewardObject = addDiamond(drawables, *envState.scene, rewardObjectCoords + Vector3{0, 1.2, 0}, {0.17f * scale, 0.35f * scale, 0.17f * scale}, ColorRgb::VIOLET);
}
