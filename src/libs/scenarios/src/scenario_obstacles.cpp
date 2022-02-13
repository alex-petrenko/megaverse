#include <set>

#include <Magnum/Math/Angle.h>

#include <scenarios/scenario_obstacles.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace Megaverse;


namespace
{

std::unique_ptr<Platform> makePlatform(
    const std::vector<PlatformType> &platformTypes, Object3D *parent, Rng &rng,
    int walls, const FloatParams &params, int width
)
{
    const auto platformType = randomSample(platformTypes, rng);

    switch (platformType) {
        case PlatformType::STEP:
            return std::make_unique<StepPlatform>(parent, rng, walls, params, width);
        case PlatformType::GAP:
            return std::make_unique<GapPlatform>(parent, rng, walls, params, width);
        case PlatformType::LAVA:
            return std::make_unique<LavaPlatform>(parent, rng, walls, params, width);
        case PlatformType::WALL:
            return std::make_unique<WallPlatform>(parent, rng, walls, params, width);
        case PlatformType::EMPTY:
        default:
            return std::make_unique<EmptyPlatform>(parent, rng, walls, params, width);
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

    agentSpawnPositions.clear(), objectSpawnPositions.clear(), rewardSpawnPositions.clear();
    agentReachedExit = std::vector<bool>(env.getNumAgents(), false);
    solved = false;

    auto &platforms = platformsComponent.platforms;

    const bool drawWalls = randRange(0, 2, envState.rng);

    StartPlatform *startPlatform = nullptr;

    // generating the level layout
    for (int attempt = 0; attempt < 20; ++attempt) {
        platforms.clear();

        numPlatforms = randRange(
            int(lround(floatParams[Str::obstaclesMinNumPlatforms])),
            int(lround(floatParams[Str::obstaclesMaxNumPlatforms])) + 1,
            envState.rng
        );

        static const std::vector<int> orientations = {ORIENTATION_STRAIGHT, ORIENTATION_TURN_LEFT, ORIENTATION_TURN_RIGHT};

        auto startPlatformPtr = std::make_unique<StartPlatform>(platformsComponent.levelRoot.get(), envState.rng, floatParams);
        startPlatformPtr->init(), startPlatformPtr->generate();
        int requiredWidth = startPlatformPtr->width;

        startPlatform = startPlatformPtr.get();
        Platform *previousPlatform = startPlatform;

        platformsComponent.addPlatform(std::move(startPlatformPtr));

        int numMaxDifficultyObstacles = 0;
        int numAllowedMaxDifficultyObstacles = int(floatParams.at(Str::obstaclesNumAllowedMaxDifficulty));

        for (int i = 0; i < numPlatforms; ++i) {
            auto orientation = randomSample(orientations, envState.rng);
            requiredWidth = orientation == ORIENTATION_STRAIGHT ? requiredWidth : -1;

            std::unique_ptr<Platform> newPlatform;
            while (!newPlatform || (newPlatform->isMaxDifficulty() && numMaxDifficultyObstacles >= numAllowedMaxDifficultyObstacles)) {
                newPlatform = makePlatform(platformTypes, previousPlatform->nextPlatformAnchor, envState.rng, WALLS_WEST | WALLS_EAST, floatParams, requiredWidth);
                newPlatform->init();
            }

            if (newPlatform->isMaxDifficulty()) {
                // TLOG(INFO) << "Max difficulty obstacle!";
                ++numMaxDifficultyObstacles;
            }

            platformsComponent.addPlatform(std::move(newPlatform));

            auto platform = platforms.back().get();
            platform->generate();

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

                platformsComponent.addPlatform(std::make_unique<TransitionPlatform>(previousPlatform->nextPlatformAnchor, envState.rng, walls, floatParams, l, w));
                auto transitionPlatform = platforms.back().get();

                transitionPlatform->init();
                transitionPlatform->generate();
            }

            previousPlatform = platform;
            requiredWidth = platform->width;
        }

        auto exitPlatformPtr = std::make_unique<ExitPlatform>(previousPlatform->nextPlatformAnchor, envState.rng, floatParams, requiredWidth);
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

    auto layoutColor = randomLayoutColor(envState.rng);
    auto wallColor = randomLayoutColor(envState.rng);
    for (auto &p : platforms)
        vg.addPlatform(*p, layoutColor, wallColor, drawWalls);

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
        float randomBoxesFraction = frand(envState.rng) * 0.5f;
        auto randomBoxes = int(lround(randomBoxesFraction * numBoxes[i])) + randRange(0, 2, envState.rng);

        const auto coords = platforms[i]->generateObjectPositions(numBoxes[i] + randomBoxes);
        objectSpawnPositions.insert(objectSpawnPositions.end(), coords.cbegin(), coords.cend());
    }

    for (int i = 1; i < int(platforms.size()) - 1; ++i) {
        auto numRewardObjects = randRange(0, 2, envState.rng);
        const auto coords = platforms[i]->generateObjectPositions(numRewardObjects);
        rewardSpawnPositions.insert(rewardSpawnPositions.end(), coords.cbegin(), coords.cend());
    }
}

void ObstaclesScenario::step()
{
    objectStackingComponent.step(env, envState);
    fallDetection.step(env, envState);

    int numAgentsAtExit = 0;
    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto agent = envState.agents[i];
        const auto t = agent->absoluteTransformation().translation();
        const auto voxel = vg.grid.getCoords(t);

        if (vg.grid.hasVoxel(voxel)) {
            const auto terrainType = vg.grid.get(voxel)->terrain;
            if (terrainType & TERRAIN_EXIT) {
                ++numAgentsAtExit;
                if (!agentReachedExit[i]) {
                    agentReachedExit[i] = true;
                    rewardTeam(Str::obstaclesAgentAtExit, i, 1);

                    if (objectStackingComponent.agentCarryingObject(i)) {
                        rewardTeam(Str::obstaclesAgentCarriedObjectToExit, i, 1);
                        // TLOG(INFO) << "Carried object to exit";
                    }
                }
            } else if (terrainType & TERRAIN_LAVA)
                agentTouchedLava(i);

            // additional reward objects promote exploration
            auto voxelData = vg.grid.get(voxel);
            if (voxelData->rewardObject) {
                voxelData->rewardObject->translate({1000, 1000, 1000});
                voxelData->rewardObject = nullptr;  // remove the reference, but the object will be later cleaned when we destroy the scene graph
                rewardTeam(Str::obstaclesExtraReward, i, 1);
            }
        }
    }

    if (numAgentsAtExit == env.getNumAgents() && !solved) {
        solved = true;
        doneWithTimer();
        rewardAll(Str::obstaclesAllAgentsAtExit, 1);
    }
}

void ObstaclesScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    addDrawablesAndCollisionObjectsFromVoxelGrid(vg, drawables, envState, 1);

    // add terrains
    for (auto &platform : platformsComponent.platforms)
        for (auto &[terrainType, boxes] : platform->terrainBoxes)
            for (auto &bb : boxes)
                addTerrain(drawables, envState, terrainType, bb.boundingBox());

    objectStackingComponent.addDrawablesAndCollisions(drawables, envState, objectSpawnPositions);

    for (const auto &pos : rewardSpawnPositions) {
        auto rewardObject = addDiamond(drawables, *envState.scene, Vector3{pos} + Vector3{0.5, 0.7, 0.5}, Vector3{0.17f, 0.45f, 0.17f} * 0.8f, ColorRgb::GREEN);
        if (!vg.grid.hasVoxel(pos))
            vg.grid.set(pos, makeVoxel<VoxelObstacles>(VOXEL_EMPTY));

        vg.grid.get(pos)->rewardObject = rewardObject;
    }
}

float ObstaclesScenario::episodeLengthSec() const
{
    const auto minDuration = Scenario::episodeLengthSec();
    return std::max(minDuration, float(numPlatforms) * 35 + float(objectSpawnPositions.size()) * 1);
}

void ObstaclesScenario::agentFell(int)
{
    // we don't penalize the agent for stepping onto lava or falling
    // otherwise they get discouraged and never even go near these obstacles
}

void ObstaclesScenario::agentTouchedLava(int agentIdx)
{
    fallDetection.resetAgent(agentIdx, envState.agents[agentIdx]);
    agentFell(agentIdx);
}
