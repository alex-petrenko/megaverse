#include <scenarios/const.hpp>
#include <scenarios/scenario_tower_building.hpp>


using namespace VoxelWorld;


class TowerBuilding::TowerBuildingPlatform : public EmptyPlatform
{
public:
    explicit TowerBuildingPlatform(Object3D *parent, Rng &rng, int walls, int numAgents)
    : EmptyPlatform(parent, rng, walls)
    , numAgents{numAgents}
    {
    }

    ~TowerBuildingPlatform() = default;

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

        agentSpawnCoords = std::vector<VoxelCoords>(
            spawnCandidates.begin(), spawnCandidates.begin() + std::min(numAgents, int(spawnCandidates.size()))
        );
        auto spawnIdx = int(agentSpawnCoords.size());

        const auto maxRandomObjects = std::min(int(spawnCandidates.size()) - numAgents, 25);
        const auto spawnObjects = randRange(0, std::max(1, maxRandomObjects), rng);

        objectSpawnCoords = std::vector<VoxelCoords>(
            spawnCandidates.begin() + spawnIdx, spawnCandidates.begin() + spawnIdx + spawnObjects
        );

        for (auto &c : objectSpawnCoords) {
            if (c.x() >= materialsXOffset && c.x() < materialsXOffset + materialsLength
                && c.z() >= materialsZOffset && c.z() < materialsZOffset + materialsWidth) {
                continue;
            }

            c.y() -= 1;  // put the object on the floor
        }

        // add the main bulk of materials
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

        MagnumAABB buildingZone{*root, {minCoord, maxCoord}};
        terrainBoxes[TERRAIN_BUILDING_ZONE].emplace_back(buildingZone);
    }

    std::vector<VoxelCoords> agentSpawnPoints(int /*numAgents*/) override
    {
        return agentSpawnCoords;
    }

    // TODO override base class
    std::vector<VoxelCoords> objectSpawnPositions()
    {
        return objectSpawnCoords;
    }

private:
    std::vector<VoxelCoords> agentSpawnCoords;
    std::vector<VoxelCoords> objectSpawnCoords;

    int buildZoneLength{}, buildZoneWidth{}, materialsLength{}, materialsWidth{};
    int buildZoneXOffset{}, buildZoneZOffset{}, materialsXOffset{}, materialsZOffset{};

    int numAgents{};
};


TowerBuilding::TowerBuilding(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario{name, env, envState}
, vg{*this}
, objectStackingComponent{*this, env.getNumAgents(), vg.grid, *this}
, platformsComponent{*this}
, agentState(size_t(env.getNumAgents()))
{
    std::map<std::string, float> rewardShapingScheme{
        {Str::teamSpirit, 0.1f},  // from 0 to 1
        {Str::pickedUpObject, 0.1f},
        {Str::visitedBuildingZoneWithObject, 0.1f},
    };

    for (int i = 0; i < env.getNumAgents(); ++i)
        rewardShaping[i] = rewardShapingScheme;
}

TowerBuilding::~TowerBuilding() = default;

void TowerBuilding::reset()
{
    objectStackingComponent.reset(env, envState);
    vg.reset(env, envState);
    platformsComponent.reset(env, envState);

    std::fill(agentState.begin(), agentState.end(), AgentState{});
    previousReward.clear();

    platform = std::make_unique<TowerBuildingPlatform>(platformsComponent.levelRoot.get(), envState.rng, WALLS_ALL, env.getNumAgents());
    platform->init(), platform->generate();
    vg.addPlatform(*platform, true);

    buildingZone = platform->terrainBoxes[TERRAIN_BUILDING_ZONE].front().boundingBox();
}

std::vector<VoxelCoords> TowerBuilding::agentStartingPositions()
{
    return platform->agentSpawnPoints(env.getNumAgents());
}

void TowerBuilding::addEpisodeDrawables(DrawablesMap &drawables)
{
    // TODO: repeating code, reuse? Move to platform component



    auto boundingBoxesByType = vg.toBoundingBoxes();
    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType);

    for (auto &[terrainType, boxes] : platform->terrainBoxes)
        for (auto &bb : boxes)
            addTerrain(drawables, envState, terrainType, bb.boundingBox());

    objectStackingComponent.addDrawablesAndCollisions(drawables, envState, platform->objectSpawnPositions());
}

void TowerBuilding::step()
{
    objectStackingComponent.step(env, envState);

    for (int i = 0; i < env.getNumAgents(); ++i) {
        // reward shaping: give agents reward for visiting bulding zone while carrying the object
        if (objectStackingComponent.agentCarryingObject(i)) {
            const auto &agent = envState.agents[i];
            const auto t = agent->transformation().translation();

            VoxelCoords voxel = toVoxel(t);
            if (isInBuildingZone(voxel)) {
                if (!agentState[i].visitedBuildingZoneWithObject) {
                    envState.lastReward[i] += rewardShaping[i].at(Str::visitedBuildingZoneWithObject);
                    agentState[i].visitedBuildingZoneWithObject = true;
                }
            }
        }
    }
}

bool TowerBuilding::canPlaceObject(int, const VoxelCoords &c, Object3D *)
{
    return isInBuildingZone(c);
}

void TowerBuilding::placedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj)
{
    int placedHeight = voxel.y();
    const auto newReward = buildingReward(float(placedHeight));

    auto rewardDelta = newReward - previousReward[obj];
    previousReward[obj] = newReward;

    TLOG(INFO) << "Curr reward: " << rewardDelta << " new reward: " << newReward;

    highestTower = std::max(highestTower, voxel.y() - buildingZone.min.y() + 1);

    addCollectiveReward(agentIdx, rewardDelta);
}

void TowerBuilding::pickedObject(int agentIdx, const VoxelCoords &, Object3D *)
{
    if (!agentState[agentIdx].pickedUpObject) {
        envState.lastReward[agentIdx] += rewardShaping[agentIdx].at(Str::pickedUpObject);
        agentState[agentIdx].pickedUpObject = true;
    }
}

bool TowerBuilding::isInBuildingZone(const VoxelCoords &c) const
{
    return c.x() >= buildingZone.min.x() && c.x() < buildingZone.max.x() && c.z() >= buildingZone.min.z() && c.z() < buildingZone.max.z();
}

float TowerBuilding::buildingReward(float height) const
{
    return height * 0.2f;
}

void TowerBuilding::addCollectiveReward(int agentIdx, float rewardDelta) const
{
    for (auto i = 0; i < env.getNumAgents(); ++i) {
        if (i == agentIdx)
            envState.lastReward[i] += rewardDelta;
        else {
            const auto teamSpirit = rewardShaping[i].at(Str::teamSpirit);
            envState.lastReward[i] += teamSpirit * rewardDelta;
        }
    }
}
