#include <random>

#include <Magnum/SceneGraph/Camera.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


Env::Env(int numAgents)
    : numAgents{numAgents}
    , currAction(size_t(numAgents), Action::Idle)
    , lastReward(size_t(numAgents), 0.0f)
{
    // what is this?
    bBroadphase.getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
}

void Env::seed(int seedValue)
{
    rng.seed((unsigned long)seedValue);
}

void Env::reset()
{
    auto seed = randRange(0, 10000, rng);
    rng.seed((unsigned long)seed);
    TLOG(INFO) << "Using seed " << seed;

    episodeDurationSec = 0;

    // delete the previous layout/state
    grid.clear();

    layoutGenerator.init();
    layoutGenerator.generateFloorWalls(grid);
    layoutGenerator.generateCave(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(numAgents);

    auto possibleStartingPositions = layoutGenerator.startingPositions(grid);
    std::shuffle(possibleStartingPositions.begin(), possibleStartingPositions.end(), rng);

    agentStartingPositions = std::vector<VoxelCoords>{possibleStartingPositions.cbegin(), possibleStartingPositions.cbegin() + numAgents};

    scene = std::make_unique<Scene3D>();

    agents.clear();
    for (int i = 0; i < numAgents; ++i) {
        auto randomRotation = frand(rng) * Magnum::Constants::pi() * 2;
        auto &agent = scene->addChild<Agent>(scene.get(), bWorld, Vector3{agentStartingPositions[i]} + Vector3{0.5, 0.5, 0.5}, randomRotation);
        agents.emplace_back(&agent);
    }

    // map layout
    {
        layoutObjects.clear();
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

            // update position of the collision shape
            // layoutObject.syncPose();

            layoutObjects.emplace_back(&layoutObject);
            collisionShapes.emplace_back(std::move(bBoxShape));
        }

//        btCollisionShape* groundShape = new btBoxShape(btVector3(5,3.5,5));
//        auto groundRigidBody = new btRigidBody(0, nullptr, groundShape);
//        bWorld.addRigidBody(groundRigidBody);
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

        // if (acceleration.length() > 0)
        //    TLOG(INFO) << "acc direction: " << acceleration.x() << " " << acceleration.y() << " " << acceleration.z();

        agent->accelerate(acceleration, lastFrameDurationSec);

        if (!!(a & Action::Jump))
            agent->jump();
    }

    bWorld.stepSimulation(lastFrameDurationSec, 1, simulationStepSeconds);

    for (auto agent : agents)
        agent->updateTransform();

    bool done = false;
    int numAgentsAtExit = 0;

    for (int i = 0; i < int(agents.size()); ++i) {
        const auto &agent = agents[i];
        const auto t = agent->transformation().translation();

        if (t.x() >= exitPad.min.x() && t.x() <= exitPad.max.x()
            && t.y() >= exitPad.min.y() && t.y() <= exitPad.max.y() + 2  // TODO: hack
            && t.z() >= exitPad.min.z() && t.z() <= exitPad.max.z()) {
            ++numAgentsAtExit;
            lastReward[i] += 0.05f;
        }
    }

    if (numAgentsAtExit == numAgents) {
        done = true;
        for (int i = 0; i < int(agents.size()); ++i)
            lastReward[i] += 5.0f;
    }

    episodeDurationSec += lastFrameDurationSec;
    if (episodeDurationSec >= horizonSec)
        done = true;

    // clear the actions
    for (int i = 0; i < numAgents; ++i)
        currAction[i] = Action::Idle;

    return done;
}
