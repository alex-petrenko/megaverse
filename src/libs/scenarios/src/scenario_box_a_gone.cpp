#include <scenarios/scenario_box_a_gone.hpp>


using namespace VoxelWorld;


class BoxAGoneScenario::BoxAGonePlatform : public EmptyPlatform
{
public:
    explicit BoxAGonePlatform(Object3D *parent, Rng &rng, int walls, int)
    : EmptyPlatform(parent, rng, walls)
    {
    }

    ~BoxAGonePlatform() = default;

    void init() override
    {
        height = 8;
        length = width = platformSize;
    }

    void generate() override
    {
        EmptyPlatform::generate();
    }
};


BoxAGoneScenario::BoxAGoneScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this, 100, 0, 0, 0, voxelSize}
, platformsComponent{*this}
, fallDetection{*this, vg.grid, *this}
{
    std::map<std::string, float> rewardShapingScheme{
        {Str::boxagoneTouchedFloor,  -0.1f},
        {Str::boxagonePerStepReward, 0.01f},
        {Str::boxagoneLastManStanding, 0.05f},
    };
    for (int i = 0; i < env.getNumAgents(); ++i)
        rewardShaping[i] = rewardShapingScheme;
}

BoxAGoneScenario::~BoxAGoneScenario() = default;

void BoxAGoneScenario::reset()
{
    vg.reset(env, envState);
    platformsComponent.reset(env, envState);
    fallDetection.reset(env, envState);

    disappearingPlatforms.clear(), spawnPositions.clear();

    agentStates = std::vector<AgentState>(env.getNumAgents());
    platformStates.clear();
    extraPlatforms.clear();

    platform = std::make_unique<BoxAGonePlatform>(platformsComponent.levelRoot.get(), envState.rng, WALLS_ALL, env.getNumAgents());
    platform->init(), platform->generate();
    vg.addPlatform(*platform, true);

    const int numLevels = randRange(2, 4, envState.rng);

    const static std::vector<ColorRgb> colors{ColorRgb::ORANGE, ColorRgb::BLUE, ColorRgb::VIOLET};
    const static auto numColors = colors.size();

    int currLevelHeight = 1;
    for (int level = 0; level < numLevels; ++level) {
        const auto color = colors[level % numColors];

        currLevelHeight += randRange(2, 4, envState.rng);

        const auto offset = platformSize / 2;
        const int levelLength = randRange(10, 19, envState.rng);
        const int levelWidth = randRange(10, 19, envState.rng);
        const int startX = offset - levelLength / 2, startZ = offset - levelWidth / 2;
        const float skipProb = frand(envState.rng) * 0.2f;  // uniformly distributed between 0% and 25%

        for (int x = startX; x < startX + levelLength; ++x)
            for (int z = startZ; z < startZ + levelWidth; ++z) {
                // skip some platforms
                if (frand(envState.rng) < skipProb)
                    continue;

                VoxelCoords coords{x, currLevelHeight, z};
                disappearingPlatforms.emplace_back(coords, color);

                if (level == numLevels - 1) {
                    Magnum::Vector3 v(float(x) + 0.5f, float(currLevelHeight) + 0.5f, float(z) + 0.5f);
                    spawnPositions.emplace_back(v * voxelSize);
                }
            }
    }

    while (int(spawnPositions.size()) < env.getNumAgents())
        spawnPositions.emplace_back(spawnPositions[0]);

    std::shuffle(spawnPositions.begin(), spawnPositions.end(), envState.rng);
}

void BoxAGoneScenario::step()
{
    int agentsTouchingFloor = 0;
    int doesNotTouchFloorIdx = -1;

    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto agent = envState.agents[i];
        const auto &t = agent->absoluteTransformation().translation();
        const auto coords = vg.grid.getCoords(t);

        const bool touchesFloor = coords.y() < 3;

        if (touchesFloor) {
            envState.lastReward[i] += rewardShaping[i].at(Str::boxagoneTouchedFloor);
            ++agentsTouchingFloor;
        }
        else {
            envState.lastReward[i] += rewardShaping[i].at(Str::boxagonePerStepReward);
            doesNotTouchFloorIdx = i;
        }

        auto voxel = vg.grid.get(coords);
        if (voxel && voxel->disappearingPlatform && agent->onGround()) {
            if (voxel->disappearingPlatform != agentStates[i].lastPlatform) {
                // visited new platform
                // set the timer for the previous visited platform to disappear
                if (platformStates.count(agentStates[i].lastPlatform)) {
                    auto &p = platformStates[agentStates[i].lastPlatform];
                    p.remainingTicks = std::min(p.remainingTicks, 3);
                }

                // add new platform state
                if (!platformStates.count(voxel->disappearingPlatform)) {
                    const static auto ticks = 15;
                    const auto temporaryPlatform = extraPlatforms.back();
                    platformStates[voxel->disappearingPlatform] = PlatformState{ticks, coords, temporaryPlatform};
                    extraPlatforms.pop_back();
                    extraPlatforms.push_front(temporaryPlatform);

                    const auto platformSc = voxel->disappearingPlatform->absoluteTransformation().scaling();
                    const auto platformTr = voxel->disappearingPlatform->absoluteTransformation().translation();
                    temporaryPlatform->resetTransformation();
                    temporaryPlatform->scale(platformSc * 1.05f);
                    temporaryPlatform->translate(platformTr);
                    temporaryPlatform->syncPose();

                    voxel->disappearingPlatform->translate(Magnum::Vector3{300, 300, 300} * voxelSize);  // basically remove from the scene
                    voxel->disappearingPlatform->syncPose();
                }

                agentStates[i].lastPlatform = voxel->disappearingPlatform;
            }
        }
    }

    for (auto iter = platformStates.begin(); iter != platformStates.end();) {
        auto &state = iter->second;
        auto &tempPlatform = state.temporaryPlatform;

        --state.remainingTicks;
        if (state.remainingTicks <= 0) {
            tempPlatform->translate(Magnum::Vector3{300, 300, 300} * voxelSize);  // basically remove from the scene
            tempPlatform->syncPose();
            platformStates.erase(iter++);
            vg.grid.remove(state.coords);
        } else {
            if (state.remainingTicks <= 5) {
                const auto platformSc = tempPlatform->absoluteTransformation().scaling();
                const auto platformTr = tempPlatform->absoluteTransformation().translation();
                tempPlatform->resetTransformation().scale(platformSc * 1.03f).translate(platformTr);
                tempPlatform->syncPose();
            }

            ++iter;
        }
    }

    if (agentsTouchingFloor == env.getNumAgents() - 1 && env.getNumAgents() > 1)
        if (doesNotTouchFloorIdx >= 0)
            envState.lastReward[doesNotTouchFloorIdx] += rewardShaping[doesNotTouchFloorIdx].at(Str::boxagoneLastManStanding);

    if (agentsTouchingFloor >= env.getNumAgents())
        envState.done = true;  // TODO: true reward
}

std::vector<Magnum::Vector3> BoxAGoneScenario::agentStartingPositions()
{
    return std::vector<Magnum::Vector3>{spawnPositions.begin(), spawnPositions.begin() + env.getNumAgents()};
}

void BoxAGoneScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();
    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType, voxelSize);

    // add terrain
    for (auto &[terrainType, boxes] : platform->terrainBoxes)
        for (auto &bb : boxes)
            addTerrain(drawables, envState, terrainType, bb.boundingBox(), voxelSize);

    addDisappearingPlatforms(drawables);
}

void BoxAGoneScenario::addDisappearingPlatforms(DrawablesMap &drawables)
{
    auto objSize = 0.42f * voxelSize;
    auto thicknessRatio = 0.045f;
    auto objScale = Magnum::Vector3{objSize, objSize * thicknessRatio, objSize};

    for (const auto &[pos, color] : disappearingPlatforms) {
        auto translation = Magnum::Vector3{float(pos.x()) + 0.5f, float(pos.y()) + 0.5f, float(pos.z()) + 0.5f} * voxelSize;

        auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});

        auto &object = envState.scene->addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
        object.scale(objScale).translate(translation);
        object.syncPose();

        drawables[DrawableType::Box].emplace_back(&object, rgb(color));

        envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));

        VoxelBoxAGone voxelState;
        voxelState.disappearingPlatform = &object;
        vg.grid.set(pos, voxelState);
    }

    for (int i = 0; i < env.getNumAgents() * 3; ++i) {
        auto translation = Magnum::Vector3{300, 300, 300} * voxelSize;
        auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});

        auto &object = envState.scene->addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
        object.scale(objScale).translate(translation);
        object.syncPose();

        drawables[DrawableType::Box].emplace_back(&object, rgb(ColorRgb::GREEN));
        envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));

        extraPlatforms.emplace_back(&object);
    }
}
