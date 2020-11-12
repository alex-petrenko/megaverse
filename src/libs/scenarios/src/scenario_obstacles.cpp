#include <set>

#include <Magnum/Math/Angle.h>

#include <scenarios/scenario_obstacles.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace VoxelWorld;


namespace
{

std::unique_ptr<Platform> makePlatform(Object3D *parent, Rng &rng, int walls, int width)
{
    enum { EMPTY, WALL, LAVA, STEP, GAP };
    const static std::vector<int> supportedPlatforms = {WALL, LAVA, STEP, GAP};
    const auto platformType = randomSample(supportedPlatforms, rng);

    switch (platformType) {
        case STEP:
            return std::make_unique<StepPlatform>(parent, rng, walls, width);
        case GAP:
            return std::make_unique<GapPlatform>(parent, rng, walls, width);
        case LAVA:
            return std::make_unique<LavaPlatform>(parent, rng, walls, width);
        case WALL:
            return std::make_unique<WallPlatform>(parent, rng, walls, width);
        case EMPTY:
        default:
            return std::make_unique<EmptyPlatform>(parent, rng, walls, width);
    }
}

}


ObstaclesScenario::ObstaclesScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this}
, platformsComponent{*this}
, objectStackingComponent{*this, env.getNumAgents(), vg.grid, *this}
, fallDetection{*this, vg.grid, *this}
{
}

void ObstaclesScenario::reset()
{
    vg.reset(env, envState);
    platformsComponent.reset(env, envState);
    objectStackingComponent.reset(env, envState);
    fallDetection.reset(env, envState);

    agentSpawnPositions.clear(), objectSpawnPositions.clear();

    auto &platforms = platformsComponent.platforms;

    const bool drawWalls = randRange(0, 2, envState.rng);

    StartPlatform *startPlatform = nullptr;

    // generating the level layout
    for (int attempt = 0; attempt < 20; ++attempt) {
        platforms.clear();

        numPlatforms = randRange(2, 8, envState.rng);

        static const std::vector<int> orientations = {ORIENTATION_STRAIGHT, ORIENTATION_TURN_LEFT, ORIENTATION_TURN_RIGHT};

        auto startPlatformPtr = std::make_unique<StartPlatform>(platformsComponent.levelRoot.get(), envState.rng);
        startPlatformPtr->init(), startPlatformPtr->generate();
        int requiredWidth = startPlatformPtr->width;

        startPlatform = startPlatformPtr.get();
        Platform *previousPlatform = startPlatform;

        platformsComponent.addPlatform(std::move(startPlatformPtr));

        for (int i = 0; i < numPlatforms; ++i) {
            auto orientation = randomSample(orientations, envState.rng);
            requiredWidth = orientation == ORIENTATION_STRAIGHT ? requiredWidth : -1;

            platformsComponent.addPlatform(makePlatform(previousPlatform->nextPlatformAnchor, envState.rng, WALLS_WEST | WALLS_EAST, requiredWidth));
            auto platform = platforms.back().get();

            platform->init(), platform->generate();

            switch (orientation) {
                case ORIENTATION_STRAIGHT:
                    break;
                case ORIENTATION_TURN_LEFT:
                    platform->rotateCCW(previousPlatform->width);
                    break;
                case ORIENTATION_TURN_RIGHT:
                    platform->rotateCW(previousPlatform->width);
                    break;
                default:
                    break;
            }

            if (orientation != ORIENTATION_STRAIGHT) {
                int walls = WALLS_NORTH;
                walls |= orientation == ORIENTATION_TURN_LEFT ? WALLS_WEST : WALLS_EAST;
                const int w = previousPlatform->width, l = platform->width - 1;

                platformsComponent.addPlatform(std::make_unique<TransitionPlatform>(previousPlatform->nextPlatformAnchor, envState.rng, walls, l, w));
                auto transitionPlatform = platforms.back().get();

                transitionPlatform->init();
                transitionPlatform->generate();
            }

            previousPlatform = platform;
            requiredWidth = platform->width;
        }

        auto exitPlatformPtr = std::make_unique<ExitPlatform>(previousPlatform->nextPlatformAnchor, envState.rng, requiredWidth);
        exitPlatformPtr->init(), exitPlatformPtr->generate();
        platformsComponent.addPlatform(std::move(exitPlatformPtr));

        // don't check collisions with self and with the previous platforms (there might be overlap)
        bool selfCollision = false;
        for (int j = 0; j < int(platforms.size()) && !selfCollision; ++j) {
            for (int k = 0; k < j - 2; ++k) {
                if (platforms[j]->collidesWith(*platforms[k])) {
                    TLOG(INFO) << "Platform " << j << " collides with " << k;
                    selfCollision = true;
                    break;
                }
            }
        }

        if (selfCollision)
            TLOG(INFO) << "Self collision! Attempt " << attempt << " re-generate!";
        else
            break;
    }

    for (auto &p : platforms)
        vg.addPlatform(*p, drawWalls);

    assert(startPlatform);
    agentSpawnPositions = startPlatform->agentSpawnPoints(env.getNumAgents());
    fallDetection.agentInitialPositions = agentSpawnPositions;

    // generating movable boxes
    std::vector<int> numBoxes(platforms.size());
    for (int i = 1; i < int(platforms.size()); ++i) {
        const auto n = platforms[i]->requiresMovableBoxesToTraverse();
        for (int box = 0; box < n; ++box) {
            const auto platformIdx = randRange(std::max(0, i - 2), i, envState.rng);
            ++numBoxes[platformIdx];
        }
    }

    for (int i = 0; i < int(platforms.size()); ++i) {
        const auto coords = platforms[i]->generateMovableBoxes(numBoxes[i]);
        objectSpawnPositions.insert(objectSpawnPositions.end(), coords.cbegin(), coords.cend());
    }
}

void ObstaclesScenario::step()
{
    objectStackingComponent.step(env, envState);
    fallDetection.step(env, envState);

    int numAgentsAtExit = 0;
    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto agent = envState.agents[i];
        const auto &t = agent->absoluteTransformation().translation();
        const auto voxel = vg.grid.getCoords(t);

        if (vg.grid.hasVoxel(voxel)) {
            const auto terrainType = vg.grid.get(voxel)->terrain;
            if (terrainType & TERRAIN_EXIT)
                ++numAgentsAtExit;
            else if (terrainType & TERRAIN_LAVA)
                agentTouchedLava(i);
        }
    }

    if (numAgentsAtExit == env.getNumAgents()) {
        envState.done = true;
        // TODO reward
    }
}

void ObstaclesScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();

    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType);

    // add terrains
    for (auto &platform : platformsComponent.platforms)
        for (auto &[terrainType, boxes] : platform->terrainBoxes)
            for (auto &bb : boxes)
                addTerrain(drawables, envState, terrainType, bb.boundingBox());

    objectStackingComponent.addDrawablesAndCollisions(drawables, envState, objectSpawnPositions);
}

float ObstaclesScenario::episodeLengthSec() const
{
    const auto minDuration = Scenario::episodeLengthSec();
    return std::max(minDuration, float(numPlatforms) * 30);
}

void ObstaclesScenario::agentFell(int agentIdx)
{
    // TODO reward
}

void ObstaclesScenario::agentTouchedLava(int agentIdx)
{
    fallDetection.resetAgent(agentIdx, envState.agents[agentIdx]);
    agentFell(agentIdx);
}
