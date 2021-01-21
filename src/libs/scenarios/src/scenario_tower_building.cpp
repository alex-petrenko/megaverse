#include <scenarios/const.hpp>
#include <scenarios/scenario_tower_building.hpp>


using namespace VoxelWorld;


class TowerBuildingScenario::TowerBuildingPlatform : public EmptyPlatform
{
public:
    explicit TowerBuildingPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int numAgents)
    : EmptyPlatform(parent, rng, walls, params)
    , numAgents{numAgents}
    {
    }

    ~TowerBuildingPlatform() override = default;

    void init() override
    {
        height = randRange(5, 7, rng);
        length = randRange(12, 30, rng);
        width = randRange(12, 25, rng);

        // determine the size and the position of the building zone
        buildZoneLength = randRange(3, 9, rng);
        buildZoneWidth = randRange(3, 9, rng);

        materialsLength = randRange(2, 8, rng);
        materialsWidth = randRange(2, 8, rng);

        length = std::max(buildZoneLength + materialsLength + 3, length);
        width = std::max(buildZoneWidth + materialsWidth + 3, width);

        buildZoneXOffset = randRange(1, length - buildZoneLength - 1, rng);
        buildZoneZOffset = randRange(1, width - buildZoneWidth - 1, rng);

        materialsXOffset = randRange(1, length - materialsLength - 1, rng);
        materialsZOffset = randRange(1, width - materialsWidth - 1, rng);

        std::vector<VoxelCoords> spawnCandidates;
        for (int x = 1; x < length - 1; ++x)
            for (int z = 1; z < width - 1; ++z)
                spawnCandidates.emplace_back(x, 2, z);

        std::shuffle(spawnCandidates.begin(), spawnCandidates.end(), rng);

        agentSpawnCoords = toFloat(std::vector<VoxelCoords>(
            spawnCandidates.begin(), spawnCandidates.begin() + std::min(numAgents, int(spawnCandidates.size()))
        ));
        auto spawnIdx = int(agentSpawnCoords.size());

        const auto maxRandomObjects = std::min(int(spawnCandidates.size()) - numAgents, 25);
        const auto spawnObjects = randRange(0, std::max(1, maxRandomObjects), rng);

        // spawn a bunch of random objects
        objectSpawnCoords = std::vector<VoxelCoords>(
            spawnCandidates.begin() + spawnIdx, spawnCandidates.begin() + spawnIdx + spawnObjects
        );

        // objects that are within the "materials" rectangle will be at y=2, otherwise drop them on the floor
        for (auto &c : objectSpawnCoords) {
            if (c.x() >= materialsXOffset && c.x() < materialsXOffset + materialsLength
                && c.z() >= materialsZOffset && c.z() < materialsZOffset + materialsWidth) {
                continue;
            }
            c.y() -= 1;  // put the object on the floor
        }

        // add the main bulk of materials, a random rectangle of boxes
        for (int x = materialsXOffset; x < materialsXOffset + materialsLength; ++x)
            for (int y = 1; y <= 1; ++y)
                for (int z = materialsZOffset; z < materialsZOffset + materialsWidth; ++z)
                    objectSpawnCoords.emplace_back(x, y, z);

        while (int(agentSpawnCoords.size()) < numAgents)
        agentSpawnCoords.emplace_back(agentSpawnCoords[0]);
    }

    void generate() override
    {
        EmptyPlatform::generate();

        const VoxelCoords minCoord{buildZoneXOffset, 1, buildZoneZOffset},
                          maxCoord{buildZoneXOffset + buildZoneLength, 1, buildZoneZOffset + buildZoneWidth};

        MagnumAABB buildingZoneBB{*root, {minCoord, maxCoord}};
        terrainBoxes[TERRAIN_BUILDING_ZONE].emplace_back(buildingZoneBB);
    }

    std::vector<Magnum::Vector3> agentSpawnPoints(int /*numAgents*/) override
    {
        return agentSpawnCoords;
    }

    std::vector<VoxelCoords> generateObjectPositions(int /*numBoxesToGenerate*/) override
    {
        return objectSpawnCoords;
    }

    int numMovableBoxes() const
    {
        return objectSpawnCoords.size();
    }

private:
    std::vector<Magnum::Vector3> agentSpawnCoords;
    std::vector<VoxelCoords> objectSpawnCoords;

    int buildZoneLength{}, buildZoneWidth{}, materialsLength{}, materialsWidth{};
    int buildZoneXOffset{}, buildZoneZOffset{}, materialsXOffset{}, materialsZOffset{};

    int numAgents{};
};


TowerBuildingScenario::TowerBuildingScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario{name, env, envState}
, vg{*this}
, objectStackingComponent{*this, env.getNumAgents(), vg.grid, *this}
, platformsComponent{*this}
, agentState(size_t(env.getNumAgents()))
{
}

TowerBuildingScenario::~TowerBuildingScenario() = default;

void TowerBuildingScenario::reset()
{
    objectStackingComponent.reset(env, envState);
    vg.reset(env, envState);
    platformsComponent.reset(env, envState);

    std::fill(agentState.begin(), agentState.end(), AgentState{});
    previousReward.clear();

    platform = std::make_unique<TowerBuildingPlatform>(platformsComponent.levelRoot.get(), envState.rng, WALLS_ALL, floatParams, env.getNumAgents());
    platform->init(), platform->generate();
    vg.addPlatform(*platform, bool(randRange(0, 2, envState.rng)));

    buildingZone = platform->terrainBoxes[TERRAIN_BUILDING_ZONE].front().boundingBox();

    currBuildingZoneReward = 0.0f;
    objectsInBuildingZone.clear();
}

std::vector<Magnum::Vector3> TowerBuildingScenario::agentStartingPositions()
{
    return platform->agentSpawnPoints(env.getNumAgents());
}

void TowerBuildingScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();
    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType);

    for (auto &[terrainType, boxes] : platform->terrainBoxes)
        for (auto &bb : boxes)
            addTerrain(drawables, envState, terrainType, bb.boundingBox());

    const auto objectPositions = platform->generateObjectPositions(-1);
    for (const auto &pos : objectPositions)
        if (isInBuildingZone(pos))
            objectsInBuildingZone.insert(pos);
    currBuildingZoneReward = calculateTowerReward();
    TLOG(INFO) << "Initial tower reward: " << currBuildingZoneReward;

    objectStackingComponent.addDrawablesAndCollisions(drawables, envState, objectPositions);
}

void TowerBuildingScenario::step()
{
    objectStackingComponent.step(env, envState);

    for (int i = 0; i < env.getNumAgents(); ++i) {
        // reward shaping: give agents reward for visiting bulding zone while carrying the object
        if (objectStackingComponent.agentCarryingObject(i)) {
            const auto &agent = envState.agents[i];
            const auto t = agent->transformation().translation();

            VoxelCoords voxel = vg.grid.getCoords(t);
            if (isInBuildingZone(voxel)) {
                if (!agentState[i].visitedBuildingZoneWithObject) {
                    rewardTeam(Str::towerVisitedBuildingZoneWithObject, i, 1);
                    agentState[i].visitedBuildingZoneWithObject = true;
                }
            }
        }
    }
}

bool TowerBuildingScenario::canPlaceObject(int, const VoxelCoords &c, Object3D *)
{
    return isInBuildingZone(c);
}

void TowerBuildingScenario::placedObject(int agentIdx, const VoxelCoords &voxel, Object3D *)
{
    if (isInBuildingZone(voxel))
        objectsInBuildingZone.insert(voxel);

    addCollectiveReward(agentIdx);

    highestTower = std::max(highestTower, voxel.y() - buildingZone.min.y() + 1);
}

void TowerBuildingScenario::pickedObject(int agentIdx, const VoxelCoords &voxel, Object3D *)
{
    if (isInBuildingZone(voxel))
        objectsInBuildingZone.erase(voxel);

    if (!agentState[agentIdx].pickedUpObject) {
        rewardAgent(Str::towerPickedUpObject, agentIdx, 1);
        agentState[agentIdx].pickedUpObject = true;
    }
}

bool TowerBuildingScenario::isInBuildingZone(const VoxelCoords &c) const
{
    return c.x() >= buildingZone.min.x() && c.x() < buildingZone.max.x() && c.z() >= buildingZone.min.z() && c.z() < buildingZone.max.z();
}

float TowerBuildingScenario::calculateTowerReward() const
{
    float reward = 0.0f;
    for (auto &pos : objectsInBuildingZone) {
        auto height = pos.y();
        reward += buildingRewardCoeffForHeight(height);
    }

    return reward;
}

/**
 * The higher the agents place the blocks the more we reward them.
 */
float TowerBuildingScenario::buildingRewardCoeffForHeight(float height)
{
    auto res = height * 0.05f;
    res += std::min(0.05f * powf(2, height), 20.0f);
    return res;
}

void TowerBuildingScenario::addCollectiveReward(int agentIdx)
{
    auto newReward = calculateTowerReward();
    auto rewardDelta = newReward - currBuildingZoneReward;
    TLOG(INFO) << "Curr reward: " << rewardDelta << " new reward: " << newReward;

    currBuildingZoneReward = newReward;
    rewardTeam(Str::towerBuildingReward, agentIdx, rewardDelta);
}

float VoxelWorld::TowerBuildingScenario::episodeLengthSec() const
{
    return Scenario::episodeLengthSec() + 4.0f * float(platform->numMovableBoxes());
}
