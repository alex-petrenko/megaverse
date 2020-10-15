#include <scenarios/const.hpp>
#include <scenarios/scenario_tower_building.hpp>


using namespace VoxelWorld;


TowerBuilding::TowerBuilding(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario{name, env, envState}
, vg{*this}
, objectStackingComponent{*this, env.getNumAgents(), vg.grid, *this}
, gridLayoutComponent{*this, envState.rng}
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

void TowerBuilding::reset()
{
    std::fill(agentState.begin(), agentState.end(), AgentState{});
    previousReward.clear();

    vg.reset(env, envState);

    gridLayoutComponent.init(env.getNumAgents(), LayoutType::Towers);
    gridLayoutComponent.generate(vg.grid);

    buildingZone = gridLayoutComponent.buildingZone(vg.grid);

    // TODO: remove
//    auto exitPad = gridLayoutComponent.levelExit(vg.grid);
//    auto exitPadCenter = 0.5f * Magnum::Vector3(exitPad.max + exitPad.min);
//    exitPadCenter.y() = exitPad.min.y();
}

std::vector<VoxelCoords> TowerBuilding::agentStartingPositions()
{
    return gridLayoutComponent.startingPositions(vg.grid);
}

void TowerBuilding::addEpisodeDrawables(DrawablesMap &drawables)
{
    gridLayoutComponent.addLayoutDrawables(drawables, envState, vg.grid, true);
}

void TowerBuilding::step()
{
    for (int i = 0; i < env.getNumAgents(); ++i) {
        const auto a = envState.currAction[i];
        if (!!(a & Action::Interact))
            objectStackingComponent.onInteractAction(i, envState);

        // reward shaping: give agents reward for visiting bulding zone while carrying the object
        if (objectStackingComponent.agentCarryingObject(i)) {
            const auto &agent = envState.agents[i];
            const auto t = agent->transformation().translation();

            VoxelCoords voxel{t};
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

    const auto newReward = buildingReward(placedHeight);

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

