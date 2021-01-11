#include <mazes/honeycombmaze.h>

#include <scenarios/layout_utils.hpp>
#include <scenarios/scenario_hex_memory.hpp>

using namespace VoxelWorld;


HexMemoryScenario::HexMemoryScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, maze{*this}
, vg{*this, 100, 0, 0, 0, 1.0}
{
    std::map<std::string, float> rewardShapingScheme{
        {Str::memoryCollectGood, 1.0f},
        {Str::memoryCollectBad, -1.0f},
    };

    for (int i = 0; i < env.getNumAgents(); ++i)
        rewardShaping[i] = rewardShapingScheme;
}

HexMemoryScenario::~HexMemoryScenario() = default;

void HexMemoryScenario::reset()
{
    vg.reset(env, envState);

    goodObjects.clear(), badObjects.clear();
    goodObjectsCollected = 0;

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
    constexpr auto collectRadius = 1.0f;

    if (goodObjectsCollected >= int(goodObjects.size()))
        envState.done = true;

    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto agent = envState.agents[i];
        const auto t = agent->absoluteTransformation().translation();
        const auto agentCoords = vg.grid.getCoords(t);

        // checking the surrounding voxels for objects we can collect
        // (that's a lot of code, isn't it easier to just check all the objects globally lol)

        for (int dx = -1; dx <= 1; ++dx)
            for (int dz = -1; dz <= 1; ++dz) {
                const VoxelCoords coords{agentCoords.x() + dx, agentCoords.y(), agentCoords.z() + dz};

                if (!vg.grid.hasVoxel(coords))
                    continue;

                const auto voxel = vg.grid.get(coords);
                for (auto it = voxel->objects.begin(); it != voxel->objects.end();) {
                    const auto pillarPos = it->object->absoluteTransformation().translation();
                    const auto distance = (pillarPos - t).length();
                    if (distance < collectRadius) {
                        // collecting the object
                        envState.lastReward[i] += it->good ? rewardShaping[i].at(Str::memoryCollectGood) : rewardShaping[i].at(Str::memoryCollectBad);
                        goodObjectsCollected += it->good;
                        it->object->translate({100, 100, 100});
                        it = voxel->objects.erase(it);
                    } else
                        ++it;
                }
            }
    }
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
    const static std::vector<ColorRgb> pillarColors{
        ColorRgb::YELLOW, ColorRgb::LIGHT_GREEN, ColorRgb::LIGHT_BLUE, ColorRgb::ORANGE,
        ColorRgb::DARK_GREY, ColorRgb::RED, ColorRgb::VIOLET,
    };

    const auto goodPillarColor = randomSample(pillarColors, envState.rng);
    ColorRgb badPillarColor = randomSample(pillarColors, envState.rng);
    while (badPillarColor == goodPillarColor)
        badPillarColor = randomSample(pillarColors, envState.rng);

    maze.addDrawablesAndCollisions(drawables, envState);

    // adding landmark object
    Magnum::Vector3 landmarkScale {0.5, 2, 0.5}, objectScale = landmarkScale * 0.5f;
    addPillar(drawables, *envState.scene, landmarkLocation, landmarkScale, goodPillarColor);

    bool isGood = true;
    for (const auto &objects : {goodObjects, badObjects}) {
        for (const auto &coord : objects) {
            const auto object = addPillar(drawables, *envState.scene, coord, objectScale, isGood ? goodPillarColor : badPillarColor);

            const auto voxelCoord = vg.grid.getCoords(coord);
            if (!vg.grid.hasVoxel(voxelCoord))
                vg.grid.set(voxelCoord, VoxelHexMemory{});

            vg.grid.get(voxelCoord)->objects.emplace_back(CollectableObject{object, isGood});
        }

        isGood = !isGood;
    }
}
