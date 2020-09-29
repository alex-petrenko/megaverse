#include <random>

#include <Magnum/SceneGraph/Camera.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


namespace {

Color3 agentColors[] = {
    0xffdd3c_rgbf,
    0x3bb372_rgbf,
    0x2eb5d0_rgbf,
    0xffa351_rgbf,
};
const int numAgentColors = ARR_LENGTH(agentColors);

const auto agentEyesColor = 0x222222_rgbf,
           layoutColor = 0xffffff_rgbf,
           movableObjectColor = 0xadd8e6_rgbf,
           exitPadColor = 0x50c878_rgbf,
           buildingZoneColor = 0x555555_rgbf;

}


/**
Left = 1 << 1,
Right = 1 << 2,

Forward = 1 << 3,
Backward = 1 << 4,

LookLeft = 1 << 5,
LookRight = 1 << 6,

Jump = 1 << 7,
Interact = 1 << 8,

LookDown = 1 << 9,
LookUp = 1 << 10,
**/
const std::vector<int> Env::actionSpaceSizes = {3, 3, 3, 2, 2, 3};


Env::Env(int numAgents, float verticalLookLimitRad)
    : totalReward(size_t(numAgents), 0.0f)
    , rewardShaping(size_t(numAgents))
    , numAgents{numAgents}
    , verticalLookLimitRad{verticalLookLimitRad}
    , currAction(size_t(numAgents), Action::Idle)
    , lastReward(size_t(numAgents), 0.0f)
{
    std::map<std::string, float> rewardShapingScheme{
        {"teamSpirit", 0.1f},  // from 0 to 1
        {"pickedUpObject", 0.1f},
        {"visitedBuildingZoneWithObject", 0.1f},
    };

    for (int i = 0; i < numAgents; ++i)
        rewardShaping[i] = rewardShapingScheme;

    // what is this?
    bBroadphase.getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

//    availableLayouts = {LayoutType::Cave, LayoutType::Walls};  // current default
//    availableLayouts = {LayoutType::Walls};  // current default
    availableLayouts = {LayoutType::Towers};  // current default

    // empty list of drawables for each supported drawable type
    for (int drawableType = int(DrawableType::First); drawableType < int(DrawableType::NumTypes); ++drawableType)
        drawables[DrawableType(drawableType)] = std::vector<SceneObjectInfo>{};
}

void Env::seed(int seedValue)
{
    rng.seed((unsigned long)seedValue);
}

void Env::reset()
{
    std::fill(totalReward.begin(), totalReward.end(), 0.0f);
    completed = false;

    auto seed = randRange(0, 1 << 30, rng);
    rng.seed((unsigned long)seed);
    TLOG(INFO) << "Using seed " << seed;

    episodeDurationSec = 0;

    // delete the previous layout/state
    grid.clear();

    const auto layoutTypeIdx = randRange(0, int(availableLayouts.size()), rng);
    currLayoutType = availableLayouts[layoutTypeIdx];

    layoutGenerator.init(numAgents, currLayoutType);
    layoutGenerator.generate(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(grid);
    exitPadCenter = 0.5f * Vector3(exitPad.max + exitPad.min);
    exitPadCenter.y() = exitPad.min.y();

    buildingZone = layoutGenerator.buildingZone(grid);
    highestTower = 0;

    agentStartingPositions = layoutGenerator.startingPositions(grid);
    objectSpawnPositions = layoutGenerator.objectSpawnPositions(grid);

    memset(objectsPerHeight.data(), 0, objectsPerHeight.size());
    for (const auto &c : objectSpawnPositions)
        if (isInBuildingZone(c))
            ++objectsPerHeight[c.y()];

    scene = std::make_unique<Scene3D>();

    // remove dangling pointers from the previous episode
    for (int drawableType = int(DrawableType::First); drawableType < int(DrawableType::NumTypes); ++drawableType)
        drawables[DrawableType(drawableType)].clear();

    // agents
    {
        agents.clear();
        agentStates.clear();

        for (int i = 0; i < numAgents; ++i) {
            auto randomRotation = frand(rng) * Magnum::Constants::pi() * 2;
            auto &agent = scene->addChild<Agent>(scene.get(), bWorld,
                                                 Vector3{agentStartingPositions[i]} + Vector3{0.5, Agent::agentHeight / 2, 0.5},
                                                 randomRotation, verticalLookLimitRad);

            agents.emplace_back(&agent);

            agentStates.emplace_back(AgentState());
            agentStates[i].minDistToGoal = (agent.transformation().translation() - exitPadCenter).length();

            auto &agentBody = agent.addChild<Object3D>(&agent);
            agentBody.scale({0.35f, 0.36f, 0.35f}).translate({0, 0.09f, 0});
            addStandardDrawable(DrawableType::Capsule, agentBody, agentColors[i % numAgentColors]);

            addStandardDrawable(DrawableType::Box, *agent.eyesObject, agentEyesColor);

            // auto &pickupSpot = agent.pickupSpot->addChild<Object3D>();
            // pickupSpot.scale({0.03f, 0.03f, 0.03f});
            // addStandardDrawable(DrawableType::Box, pickupSpot, 0x222222_rgbf);
        }

        assert(agents.size() == agentStates.size());
    }

    // map layout
    {
        collisionShapes.clear();

        TLOG(INFO) << "Env has " << layoutDrawables.size() << " layout drawables";

        for (auto layoutDrawable : layoutDrawables) {
            const auto bboxMin = layoutDrawable.min, bboxMax = layoutDrawable.max;
            auto scale = Vector3{
                float(bboxMax.x() - bboxMin.x() + 1) / 2,
                float(bboxMax.y() - bboxMin.y() + 1) / 2,
                float(bboxMax.z() - bboxMin.z() + 1) / 2,
            };

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{scale.x(), scale.y(), scale.z()});
            // auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});
            auto &layoutObject = scene->addChild<RigidBody>(scene.get(), 0.0f, bBoxShape.get(), bWorld);

            auto translation = Magnum::Vector3{float((bboxMin.x() + bboxMax.x())) / 2 + 0.5f, float((bboxMin.y() + bboxMax.y())) / 2 + 0.5f, float((bboxMin.z() + bboxMax.z())) / 2 + 0.5f};
            layoutObject.scale(scale).translate(translation);
            layoutObject.syncPose();

            addStandardDrawable(DrawableType::Box, layoutObject, layoutColor);
            collisionShapes.emplace_back(std::move(bBoxShape));
        }

        const auto objSize = 0.39f;
        auto objScale = Vector3{objSize, objSize, objSize};

        for (const auto &movableObject : objectSpawnPositions) {
            const auto pos = movableObject;
            auto translation = Magnum::Vector3{float(pos.x()) + 0.5f, float(pos.y()) + 0.5f, float(pos.z()) + 0.5f};

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{0.45f, 0.5f, 0.45f});

            auto &object = scene->addChild<RigidBody>(scene.get(), 0.0f, bBoxShape.get(), bWorld);
            object.scale(objScale).translate(translation);
            object.setCollisionOffset({0.0f, -0.1f, 0.0f});
            object.syncPose();

            addStandardDrawable(DrawableType::Box, object, movableObjectColor);

            collisionShapes.emplace_back(std::move(bBoxShape));

            VoxelState voxelState{false};
            voxelState.obj = &object;
            grid.set(pos, voxelState);
        }

        // exit pad
        {
            const auto exitPadCoords = exitPad;
            const auto exitPadScale = Vector3(
                exitPadCoords.max.x() - exitPadCoords.min.x(),
                1.0,
                exitPadCoords.max.z() - exitPadCoords.min.z()
            );

            if (exitPadScale.x() > 0) {
                // otherwise we don't need the exit pad
                const auto exitPadPos = Vector3(exitPadCoords.min.x() + exitPadScale.x() / 2, exitPadCoords.min.y(),
                                                exitPadCoords.min.z() + exitPadScale.z() / 2);

                auto &exitPadObject = scene->addChild<Object3D>(scene.get());
                exitPadObject.scale({0.5, 0.025, 0.5}).scale(exitPadScale);
                exitPadObject.translate({0.0, 0.025, 0.0});
                exitPadObject.translate(exitPadPos);

                addStandardDrawable(DrawableType::Box, exitPadObject, exitPadColor);
            }
        }

        // building zone
        {
            const auto zoneScale = Vector3(buildingZone.max.x() - buildingZone.min.x(), 1.0, buildingZone.max.z() - buildingZone.min.z());

            if (zoneScale.x() > 0) {
                // otherwise we don't need the zone
                const auto zonePos = Vector3(buildingZone.min.x() + zoneScale.x() / 2, buildingZone.min.y(), buildingZone.min.z() + zoneScale.z() / 2);

                auto &zoneObject = scene->addChild<Object3D>(scene.get());
                zoneObject.scale({0.55, 0.075, 0.55}).scale(zoneScale);
                zoneObject.translate({0.0, 0.055, 0.0});
                zoneObject.translate(zonePos);

                addStandardDrawable(DrawableType::Box, zoneObject, buildingZoneColor);
            }
        }
    }
}

void Env::addStandardDrawable(DrawableType type, Object3D &object, const Magnum::Color3 &color)
{
    drawables[type].emplace_back(&object, color);
}

void Env::setAction(int agentIdx, Action action)
{
    currAction[agentIdx] = action;
}

void Env::step()
{
    std::fill(lastReward.begin(), lastReward.end(), 0.0f);

    for (int i = 0; i < numAgents; ++i) {
        const auto a = currAction[i];
        const auto &agent = agents[i];

        auto acceleration = btVector3{0, 0, 0};

        if (!!(a & Action::Forward))
            acceleration += agent->forwardDirection();
        else if (!!(a & Action::Backward))
            acceleration -= agent->forwardDirection();

        if (!!(a & Action::Left))
            acceleration += agent->strafeLeftDirection();
        else if (!!(a & Action::Right))
            acceleration -= agent->strafeLeftDirection();

        if (!!(a & Action::LookLeft))
            agent->lookLeft(lastFrameDurationSec);
        else if (!!(a & Action::LookRight))
            agent->lookRight(lastFrameDurationSec);

        if (!!(a & Action::LookUp))
            agent->lookUp(lastFrameDurationSec);
        else if (!!(a & Action::LookDown))
            agent->lookDown(lastFrameDurationSec);

        // if (acceleration.length() > 0)
        //    TLOG(INFO) << "acc direction: " << acceleration.x() << " " << acceleration.y() << " " << acceleration.z();

        agent->accelerate(acceleration, lastFrameDurationSec);

        if (!!(a & Action::Jump))
            agent->jump();

        if (!!(a & Action::Interact))
            objectInteract(agent, i);
    }

    bWorld.stepSimulation(lastFrameDurationSec, 1, simulationStepSeconds);

    for (auto agent : agents)
        agent->updateTransform();

    done = false;
    int numAgentsAtExit = 0;

    for (int i = 0; i < int(agents.size()); ++i) {
        const auto &agent = agents[i];
        const auto t = agent->transformation().translation();

        const auto distToGoal = (t - exitPadCenter).length();
        const auto distIncrement = 0.5f;
        const auto rewardForGettingCloserToGoal = 0.0f;

        if (agentStates[i].minDistToGoal - distToGoal > distIncrement) {
            agentStates[i].minDistToGoal -= distIncrement;
            lastReward[i] += rewardForGettingCloserToGoal;
        }

        if (t.x() >= exitPad.min.x() && t.x() <= exitPad.max.x()
            && t.y() >= exitPad.min.y() && t.y() <= exitPad.max.y() + 2  // TODO: hack
            && t.z() >= exitPad.min.z() && t.z() <= exitPad.max.z()) {
            ++numAgentsAtExit;

            if (!agentStates[i].visitedExit) {
                lastReward[i] += 3.0f;
                agentStates[i].visitedExit = true;
            }
        }

        if (agent->carryingObject) {
            VoxelCoords voxel{t};
            if (isInBuildingZone(voxel)) {
                if (!agentStates[i].visitedBuildingZoneWithObject) {
                    lastReward[i] += rewardShaping[i]["visitedBuildingZoneWithObject"];
                    agentStates[i].visitedBuildingZoneWithObject = true;
                }
            }
        }

    }

    if (numAgentsAtExit == numAgents) {
        done = true;
        completed = true;

        for (int i = 0; i < int(agents.size()); ++i) {
            lastReward[i] += 5.0f;
            if (agents[i]->carryingObject)
                lastReward[i] += 2.0f;  // TODO: this is experimental - encourage agents to carry their cubes to the exit
        }
    }

    episodeDurationSec += lastFrameDurationSec;

    if (episodeDurationSec >= horizonSec)
        done = true;

    // clear the actions
    for (int i = 0; i < numAgents; ++i)
        currAction[i] = Action::Idle;

    for (int i = 0; i < int(agents.size()); ++i)
        totalReward[i] += lastReward[i];

//    if (fabs(lastReward[0]) > SIMD_EPSILON)
//        TLOG(INFO) << "Last reward: " << lastReward[0];
}

void Env::objectInteract(Agent *agent, int agentIdx)
{
    const auto carryingScale = 0.78f, carryingScaleInverse = 1.0f / carryingScale;

    float rewardDelta = 0.0f;

    // putting object on the ground
    if (agent->carryingObject) {
        auto obj = agent->carryingObject;
        auto t = obj->absoluteTransformation().translation();

        VoxelCoords voxel{t};
        auto voxelPtr = grid.get(voxel);

        bool collidesWithAgent = false;

        for (const auto a : agents) {
            if (a == agent)
                continue;

            const auto agentTransformation = a->transformation().translation();
            VoxelCoords c{agentTransformation};

            if (voxel == c) {
                collidesWithAgent = true;
                break;
            }
        }

        if (isInBuildingZone(voxel) && !voxelPtr && !collidesWithAgent) {
            // voxel in front of us is empty, can place the object
            // the object should be on the ground or on top of another object
            // descend on y axis until we find ground

            while (true) {
                VoxelCoords voxelBelow{voxel.x(), voxel.y() - 1, voxel.z()};
                if (voxelBelow.y() < 0) {
                    // this is the lowest level we support
                    break;
                }

                auto voxelBelowPtr = grid.get(voxelBelow);
                if (voxelBelowPtr)
                    break;
                else
                    voxel = voxelBelow;
            }

            // placing object on the ground (or another object)
            VoxelState voxelState{false};
            voxelState.obj = obj;
            grid.set(voxel, voxelState);

            obj->setParent(scene.get());

            auto scaling = obj->transformation().scaling();
            obj->resetTransformation();
            obj->scale({scaling.x() * carryingScaleInverse, scaling.y() * carryingScaleInverse, scaling.z() * carryingScaleInverse});
            obj->translate({float(voxel.x()) + 0.5f, float(voxel.y()) + 0.5f, float(voxel.z()) + 0.5f});
            obj->syncPose();

            int placedHeight = voxel.y();
            bool placedInBuildingZone = isInBuildingZone(voxel);

            rewardDelta -= agent->carryingObject->placedReward;
            const auto objectsAtThisHeight = objectsPerHeight[placedHeight];
            const auto newReward = placedInBuildingZone ? buildingReward(placedHeight, objectsAtThisHeight) : 0;

//            TLOG(INFO) << "Prev reward: " << agent->carryingObject->placedReward << " Curr reward: " << newReward;

            agent->carryingObject->placedReward = newReward;
            rewardDelta += newReward;
            agent->carryingObject = nullptr;
            obj->toggleCollision();

            if (placedInBuildingZone) {
                ++objectsPerHeight[placedHeight];
                highestTower = std::max(highestTower, voxel.y() - buildingZone.min.y() + 1);
            }

//            std::ostringstream s;
//            for (int i = 0; i < 5; ++i)
//                s << objectsPerHeight[i] << " ";
//            TLOG(INFO) << s.str();
        }

    } else {
        // picking up an object

        const auto &pickup = agent->pickupSpot->absoluteTransformation().translation();
        VoxelCoords voxel{int(pickup.x()), int(pickup.y()), int(pickup.z())};
        VoxelCoords voxelAbove{voxel.x(), voxel.y() + 1, voxel.z()};

        int pickupHeight = 0, maxPickupHeight = 1;
        while (pickupHeight <= maxPickupHeight) {
            auto voxelPtr = grid.get(voxel), voxelAbovePtr = grid.get(voxelAbove);
            bool hasObjectAbove = voxelAbovePtr && voxelAbovePtr->obj;

            if (voxelPtr && voxelPtr->obj && !hasObjectAbove) {
                auto obj = voxelPtr->obj;
                obj->toggleCollision();

                obj->setParent(agent);
                auto scaling = obj->transformation().scaling();
                obj->resetTransformation();

                obj->scale({scaling.x() * carryingScale, scaling.y() * carryingScale, scaling.z() * carryingScale});
                obj->translate({0.0f, -0.3f, 0.0f});
                obj->setParent(agent->pickupSpot);

                agent->carryingObject = obj;
                if (isInBuildingZone(voxel))
                    --objectsPerHeight[voxel.y()];
                // TLOG(INFO) << "pick up height: " << agent->carryingObject->pickedUpHeight;

                grid.remove(voxel);

                if (!agentStates[agentIdx].pickedUpObject) {
                    lastReward[agentIdx] += rewardShaping[agentIdx]["picedkUpObject"];
                    agentStates[agentIdx].pickedUpObject = true;
                }

                break;

            } else {
                voxel = voxelAbove;
                voxelAbove = VoxelCoords{voxel.x(), voxel.y() + 1, voxel.z()};
            }

            ++pickupHeight;
        }
    }

    // collective rewards
    for (auto i = 0; i < numAgents; ++i) {
        if (i == agentIdx)
            lastReward[i] += rewardDelta;
        else {
            const auto teamSpirit = rewardShaping[i]["teamSpirit"];
            lastReward[i] += teamSpirit * rewardDelta;
        }
    }
}

bool Env::isInBuildingZone(const VoxelCoords &c) const
{
    return c.x() >= buildingZone.min.x() && c.x() < buildingZone.max.x() && c.z() >= buildingZone.min.z() && c.z() < buildingZone.max.z();
}

float Env::buildingReward(float height, int objectsAtThisHeight) const
{
    auto reward = std::min(0.1f * pow(2.0f, height), 10.0f);
    reward /= (1 << objectsAtThisHeight);
    return reward;
}

std::vector<Magnum::Color3> Env::getPalette() const
{
    std::vector<Color3> palette {
        agentEyesColor,
        layoutColor,
        movableObjectColor,
        exitPadColor,
        buildingZoneColor,
    };

    for (auto &agentColor : agentColors)
        palette.emplace_back(agentColor);

    return palette;
}
