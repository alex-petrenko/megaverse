#include <random>

#include <Magnum/SceneGraph/Camera.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


Env::Env(int numAgents, float verticalLookLimitRad)
    : numAgents{numAgents}
    , verticalLookLimitRad{verticalLookLimitRad}
    , currAction(size_t(numAgents), Action::Idle)
    , lastReward(size_t(numAgents), 0.0f)
{
    // what is this?
    bBroadphase.getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

//    availableLayouts = {LayoutType::Cave, LayoutType::Walls};  // current default
    availableLayouts = {LayoutType::Walls};  // current default
}

void Env::seed(int seedValue)
{
    rng.seed((unsigned long)seedValue);
}

void Env::reset()
{
    completed = false;

    auto seed = randRange(0, 1 << 30, rng);
    rng.seed((unsigned long)seed);
    TLOG(INFO) << "Using seed " << seed;

    episodeDurationSec = 0;

    // delete the previous layout/state
    grid.clear();

    const auto layoutTypeIdx = randRange(0, int(availableLayouts.size()), rng);
    const auto layoutType = availableLayouts[layoutTypeIdx];

    layoutGenerator.init(numAgents, layoutType);
    layoutGenerator.generate(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(grid);
    exitPadCenter = 0.5f * Vector3(exitPad.max + exitPad.min);
    exitPadCenter.y() = exitPad.min.y();

    agentStartingPositions = layoutGenerator.startingPositions(grid);
    objectSpawnPositions = layoutGenerator.objectSpawnPositions(grid);

    scene = std::make_unique<Scene3D>();

    agents.clear();
    agentStates.clear();

    for (int i = 0; i < numAgents; ++i) {
        auto randomRotation = frand(rng) * Magnum::Constants::pi() * 2;
        auto &agent = scene->addChild<Agent>(scene.get(), bWorld, Vector3{agentStartingPositions[i]} + Vector3{0.5, 0.55, 0.5}, randomRotation, verticalLookLimitRad);
        agents.emplace_back(&agent);

        agentStates.emplace_back(AgentState());
        agentStates[i].minDistToGoal = (agent.transformation().translation() - exitPadCenter).length();
    }

    assert(agents.size() == agentStates.size());

    // map layout
    {
        // remove dangling pointers from the previous episode
        layoutObjects.clear();
        movableObjects.clear();
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
            layoutObject.translate(translation);
            layoutObject.syncPose();
            layoutObject.translate(-translation).scale(scale).translate(translation);

            layoutObjects.emplace_back(&layoutObject);
            collisionShapes.emplace_back(std::move(bBoxShape));
        }

        const auto objSize = 0.39f;
        auto objScale = Vector3{objSize, objSize, objSize};

        for (const auto &movableObject : objectSpawnPositions) {
            const auto pos = movableObject;
            auto translation = Magnum::Vector3{float(pos.x()) + 0.5f, float(pos.y()) + 0.5f, float(pos.z()) + 0.5f};

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{objSize, objSize, objSize});

            auto &object = scene->addChild<RigidBody>(scene.get(), 0.0f, bBoxShape.get(), bWorld);
            object.scale(objScale).translate(translation);
            object.syncPose();

            movableObjects.emplace_back(&object);
            collisionShapes.emplace_back(std::move(bBoxShape));

            VoxelState voxelState{false};
            voxelState.obj = &object;
            grid.set(pos, voxelState);
        }
    }
}

void Env::setAction(int agentIdx, Action action)
{
    currAction[agentIdx] = action;
}

bool Env::step()
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
            objectInteract(agent);
    }

    bWorld.stepSimulation(lastFrameDurationSec, 1, simulationStepSeconds);

    for (auto agent : agents)
        agent->updateTransform();

    bool done = false;
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
    }

    if (numAgentsAtExit == numAgents) {
        done = true;
        completed = true;

        for (int i = 0; i < int(agents.size()); ++i) {
            lastReward[i] += 5.0f;
            if (agents[i]->carryingObject)
                lastReward[i] += 2.0f;  // TODO: this is just experimental - encourage agents to carry their cubes to the exit
        }
    }

    episodeDurationSec += lastFrameDurationSec;
    if (episodeDurationSec >= horizonSec)
        done = true;

    // clear the actions
    for (int i = 0; i < numAgents; ++i)
        currAction[i] = Action::Idle;

    return done;
}

void Env::objectInteract(Agent *agent)
{
    auto agentTransform = agent->ghostObject.getWorldTransform();
    const auto carryingScale = 0.8f, carryingScaleInverse = 1.0f / carryingScale;

    if (agent->carryingObject) {
        auto obj = agent->carryingObject;
        auto t = obj->absoluteTransformation().translation();

        VoxelCoords voxel{t};
        auto voxelPtr = grid.get(voxel);

        if (!voxelPtr) {
            VoxelState voxelState{false};
            voxelState.obj = obj;
            grid.set(voxel, voxelState);

            obj->setParent(scene.get());

            auto scaling = obj->transformation().scaling();
            obj->resetTransformation();
            obj->scale({scaling.x() * carryingScaleInverse, scaling.y() * carryingScaleInverse, scaling.z() * carryingScaleInverse});
            obj->translate({float(voxel.x()) + 0.5f, float(voxel.y()) + 0.5f, float(voxel.z()) + 0.5f});
            obj->syncPose();

            agent->carryingObject = nullptr;
            obj->toggleCollision();
        }

    } else {
        auto forwardDirection = agent->forwardDirection();

        auto interactPos = agentTransform.getOrigin() + forwardDirection.normalized();

        VoxelCoords voxel{int(interactPos.x()), int(interactPos.y()), int(interactPos.z())};

//        TLOG(INFO) << "interacting with voxel " << interactPos.x() << " " << interactPos.y() << " " << interactPos.z()
//                   << " Voxel: " << voxel;

        auto voxelPtr = grid.get(voxel);
        if (voxelPtr && voxelPtr->obj) {
            auto obj = voxelPtr->obj;
            obj->toggleCollision();

            obj->setParent(agent);
            auto scaling = obj->transformation().scaling();
            obj->resetTransformation();

            obj->scale({scaling.x() * carryingScale, scaling.y() * carryingScale, scaling.z() * carryingScale});
            obj->translate({0.0f, -0.29f, -0.9f});

            agent->carryingObject = obj;

            grid.remove(voxel);
        }
    }
}
