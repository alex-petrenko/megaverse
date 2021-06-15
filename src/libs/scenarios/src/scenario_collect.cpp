#include <util/perlin_noise.hpp>

#include <scenarios/scenario_collect.hpp>


using namespace Megaverse;

using namespace Magnum::Math::Literals;

CollectScenario::CollectScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this}
, objectStackingComponent{*this, env.getNumAgents(), vg.grid, *this}
, fallDetection{*this, vg.grid, *this}
{
}

CollectScenario::~CollectScenario() = default;

void CollectScenario::reset()
{
    solved = false;

    vg.reset(env, envState);
    objectStackingComponent.reset(env, envState);
    fallDetection.reset(env, envState);

    numPositiveRewards = positiveRewardsCollected = 0;

    createLandscape();

    fallDetection.agentInitialPositions = agentPositions;
}

void CollectScenario::createLandscape()
{
    auto &rng = envState.rng;

    static const std::vector<ColorRgb> landscapeColors = {
        ColorRgb::LAYOUT_DEFAULT,
        ColorRgb::VERY_LIGHT_GREEN,
        ColorRgb::VERY_LIGHT_BLUE,
        ColorRgb::VERY_LIGHT_GREY,
        ColorRgb::VERY_LIGHT_ORANGE,
        ColorRgb::GREY,
        ColorRgb::DARK_GREY,
    };
    static const std::vector<ColorRgb> floorColors = {
        ColorRgb::GREY,
        ColorRgb::DARK_GREY,
        ColorRgb::DARK_GREY,
    };

    auto landscapeColor = randomSample(landscapeColors, rng);
    auto floorColor = randomSample(floorColors, rng);

    constexpr static int maxWidth = 42, maxLength = maxWidth;

    const int width = randRange(8, maxWidth, rng);
    const int length = randRange(8, maxWidth, rng);

    std::vector<int> spawnHeight(length * width, 1);

    double frequency = double (randRange(1, 100, rng)) / 10.0;
    // frequency = std::clamp(frequency, 0.1, 64.0);

    const std::int32_t octaves = randRange(1, 10, rng);
    // octaves = std::clamp(octaves, 1, 16);

    const std::uint32_t seed = randRange(0, 1000000000, rng);

    const siv::PerlinNoise perlin(seed);
    const double fx = maxLength / frequency;
    const double fz = maxWidth / frequency;

    const int intensity = randRange(5, 18, rng);
    const float groundLevel = Megaverse::frand(rng) * 0.5f + 0.2f;

    for (int x = 1; x < length - 1; ++x)
        for (int z = 1; z < width - 1; ++z) {
            const double noise = perlin.accumulatedOctaveNoise2D_0_1(x / fx, z / fz, octaves);
            const double yCoord = intensity * (noise - groundLevel);

            if (yCoord >= 1) {
                const int yCoordRound = int(lround(yCoord));
                for (int y = yCoordRound; y >= 1; --y) {
                    VoxelCoords v{x, y, z};
                    vg.grid.set(v, makeVoxel<VoxelCollect>(VOXEL_SOLID | VOXEL_OPAQUE, TERRAIN_NONE, landscapeColor));
                }

                spawnHeight[x * width + z] = yCoordRound + 1;
            }
        }

    // floor
    for (int x = 0; x < length; ++x)
        for (int z = 0; z < width; ++z)
            vg.grid.set({x, 0, z}, makeVoxel<VoxelCollect>(VOXEL_SOLID | VOXEL_OPAQUE, TERRAIN_NONE, floorColor));

    std::vector<VoxelCoords> spawnPositions;

    for (int x = 1; x < length - 1; ++x)
        for (int z = 1; z < width - 1; ++z) {
            const int y = spawnHeight[x * width + z];
            spawnPositions.emplace_back(x, y, z);
        }

    std::shuffle(spawnPositions.begin(), spawnPositions.end(), rng);

    int offset = 0;
    auto agentIntPositions = std::vector<VoxelCoords>(spawnPositions.begin() + offset, spawnPositions.begin() + offset + env.getNumAgents());
    agentPositions = toFloat(agentIntPositions);
    offset += env.getNumAgents();

    int numRewards = randRange(1, int(lround(0.05 * width * length)) + 2, rng);
    numRewards = std::min(numRewards, int(spawnPositions.size()) - offset);
    int numRewardsPlacedRandomly = std::max(numRewards / 2, 1);

    // place half of the reward randomly
    rewardPositions = std::vector<VoxelCoords>(spawnPositions.begin() + offset, spawnPositions.begin() + offset + numRewardsPlacedRandomly);
    offset += numRewardsPlacedRandomly;

    std::sort(spawnPositions.begin() + offset, spawnPositions.end(), [&](const VoxelCoords &a, const VoxelCoords &b) {
        int heightA = spawnHeight[a.x() * width + a.z()];
        int heightB = spawnHeight[b.x() * width + b.z()];
        if (heightA != heightB)
            return heightA > heightB;
        else
            return false;  // equal
    });

    rewardPositions.insert(rewardPositions.end(), spawnPositions.begin() + offset, spawnPositions.begin() + offset + (numRewards - numRewardsPlacedRandomly));
    offset += numRewards - numRewardsPlacedRandomly;

    std::shuffle(spawnPositions.begin() + offset, spawnPositions.end(), rng);
    auto objectsMin = std::max(3, int(length * width * 0.04));
    auto objectsMax = std::min(objectsMin + 1, int(lround(0.07 * width * length)) + 2);
    const int numObjects = std::min(randRange(objectsMin, objectsMax, rng), int(spawnPositions.size()) - offset);
    if (offset + numObjects < int(spawnPositions.size())) {
        objectPositions = std::vector<VoxelCoords>(spawnPositions.begin() + offset, spawnPositions.begin() + offset + numObjects);
        offset += numObjects;
    }
}

void CollectScenario::step()
{
    objectStackingComponent.step(env, envState);
    fallDetection.step(env, envState);

    for (int i = 0; i < env.getNumAgents(); ++i) {
        const auto agent = envState.agents[i];
        auto t = agent->absoluteTransformation().translation();
        VoxelCoords voxel = vg.grid.getCoords(t);
        auto voxelPtr = vg.grid.get(voxel);
        if (voxelPtr && voxelPtr->rewardObject) {
            // TLOG(INFO) << "Reward " << voxelPtr->reward;
            Magnum::Vector3 far = {500, 500, 500};
            voxelPtr->rewardObject->translate(far);

            if (voxelPtr->reward > 0)
                ++positiveRewardsCollected;

            if (voxelPtr->reward > 0)
                rewardTeam(Str::collectSingleGood, i, 1);
            else if (voxelPtr->reward < 0)
                rewardTeam(Str::collectSingleBad, i, 1);

            if (positiveRewardsCollected >= numPositiveRewards && !solved) {
                TLOG(INFO) << "All rewards collected!";
                solved = true;
                doneWithTimer();
                rewardTeam(Str::collectAll, i, 1);
            }

            vg.grid.remove(voxel);
        }
    }
}

std::vector<Magnum::Vector3> CollectScenario::agentStartingPositions()
{
    return agentPositions;
}

void CollectScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    addDrawablesAndCollisionObjectsFromVoxelGrid(vg, drawables, envState, 1);

    objectStackingComponent.addDrawablesAndCollisions(drawables, envState, objectPositions);

    // adding reward objects
    for (const auto &pos : rewardPositions) {
        auto translation = Magnum::Vector3{float(pos.x()) + 0.5f, float(pos.y()) + 0.8f, float(pos.z()) + 0.5f};

        auto voxel = makeVoxel<VoxelCollect>(VOXEL_EMPTY);
        ColorRgb color;

        if (frand(envState.rng) > 0.3f) {
            voxel.reward = +1;
            color = ColorRgb::GREEN;
            ++numPositiveRewards;
        } else {
            voxel.reward = -1;
            color = ColorRgb::RED;
        }

        auto rewardObject = addDiamond(drawables, *envState.scene, translation, {0.17f, 0.45f, 0.17f}, color);
        voxel.rewardObject = rewardObject;

        vg.grid.set(pos, voxel);
    }
}

void CollectScenario::agentFell(int agentIdx)
{
    // this is just to help agents learn a bit faster
    rewardAgent(Str::collectSingleBad, agentIdx, 1);
}
