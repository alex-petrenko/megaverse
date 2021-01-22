#include <mazes/honeycombmaze.h>

#include <scenarios/layout_utils.hpp>
#include <scenarios/scenario_hex_memory.hpp>

using namespace Magnum;

using namespace VoxelWorld;


HexMemoryScenario::HexMemoryScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, maze{*this}
, vg{*this, 100, 0, 0, 0, 1.0}
{
}

HexMemoryScenario::~HexMemoryScenario() = default;

void HexMemoryScenario::reset()
{
    solved = false;

    vg.reset(env, envState);

    goodObjects.clear(), badObjects.clear();
    goodObjectsCollected = 0;

    maze.minSize = 2, maze.maxSize = 10;
    maze.omitWallsProbabilityMin = 0.1f, maze.omitWallsProbabilityMax = 0.95f;
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

    float cellWithObjectsFraction = frand(envState.rng) * 0.25f + 0.2f;
    long numCellsWithGoodObjects = std::lround(ceilf(cellWithObjectsFraction * objectCoordinates.size()));
    long numCellsWithBadObjects = std::lround(ceilf(cellWithObjectsFraction * objectCoordinates.size()));

    goodObjects = std::vector<Magnum::Vector3>(objectCoordinates.begin(), objectCoordinates.begin() + numCellsWithGoodObjects);
    if (int(objectCoordinates.size()) >= numCellsWithGoodObjects + numCellsWithBadObjects)
        badObjects = std::vector<Magnum::Vector3>(objectCoordinates.begin() + numCellsWithGoodObjects, objectCoordinates.begin() + numCellsWithGoodObjects + numCellsWithBadObjects);
}

void HexMemoryScenario::step()
{
    constexpr auto collectRadius = 1.0f;

    if (goodObjectsCollected >= int(goodObjects.size()) && !solved) {
        solved = true;
        doneWithTimer();
    }

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
                        rewardTeam(it->good ? Str::memoryCollectGood : Str::memoryCollectBad, i, 1);

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
    enum ShapeType {
        SHAPE_PILLAR,
        SHAPE_DIAMOND,
        SHAPE_SPHERE,
    };
    const static std::vector<ShapeType> shapes = {SHAPE_PILLAR, SHAPE_DIAMOND, SHAPE_SPHERE};

    auto goodObjectColor = randomObjectColor(envState.rng), badObjectColor = goodObjectColor;
    auto goodShape = randomSample(shapes, envState.rng), badShape = goodShape;

    while (badObjectColor == goodObjectColor && badShape == goodShape) {
        badObjectColor = randomObjectColor(envState.rng);
        badShape = randomSample(shapes, envState.rng);
    }

    maze.addDrawablesAndCollisions(drawables, envState);

    auto addObject = [&](ShapeType shape, ColorRgb color, const Magnum::Vector3 &loc, const Magnum::Vector3 &scale) -> Object3D * {
        switch (shape) {
            case SHAPE_SPHERE:
                return addSphere(drawables, *envState.scene, loc, scale, color);
            case SHAPE_DIAMOND:
                return addDiamond(drawables, *envState.scene, loc, scale, color);
            case SHAPE_PILLAR:
            default:
                return addPillar(drawables, *envState.scene, loc, scale, color);
        }
    };

    std::map<ShapeType, Vector3> scale {
        {SHAPE_SPHERE, {0.75, 0.75, 0.75}},
        {SHAPE_PILLAR, {0.5, 2, 0.5}},
        {SHAPE_DIAMOND, Vector3 {0.17f, 0.45f, 0.17f} * 2.2},
    };
    std::map<ShapeType, Vector3> shift {
        {SHAPE_SPHERE, {0, 0.1, 0}},
        {SHAPE_PILLAR, {0, 0.05, 0}},
        {SHAPE_DIAMOND, {0, 0.6, 0}},
    };

    // adding landmark object
    addObject(goodShape, goodObjectColor, landmarkLocation + shift[goodShape], scale[goodShape]);
    float objScale = 0.6;

    bool isGood = true;
    for (const auto &objects : {goodObjects, badObjects}) {
        for (const auto &coord : objects) {
            auto shape = isGood ? goodShape : badShape;
            auto color = isGood ? goodObjectColor : badObjectColor;

            const auto object = addObject(shape, color, coord + shift[shape] * objScale, scale[shape] * objScale);

            const auto voxelCoord = vg.grid.getCoords(coord);
            if (!vg.grid.hasVoxel(voxelCoord))
                vg.grid.set(voxelCoord, VoxelHexMemory{});

            vg.grid.get(voxelCoord)->objects.emplace_back(CollectableObject{object, isGood});
        }

        isGood = !isGood;
    }
}
