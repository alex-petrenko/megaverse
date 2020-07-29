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

    episodeDuration = 0;

    // delete the previous layout/state
    grid.clear();

    layoutGenerator.init();
    layoutGenerator.generateFloorWalls(grid);
//    layoutGenerator.generateCave(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(numAgents);

    auto possibleStartingPositions = layoutGenerator.startingPositions();
    std::shuffle(possibleStartingPositions.begin(), possibleStartingPositions.end(), rng);

    agentStartingPositions = std::vector<VoxelCoords>{possibleStartingPositions.cbegin(), possibleStartingPositions.cbegin() + numAgents};

    scene = std::make_unique<Scene3D>();

    agents.clear();
    for (int i = 0; i < numAgents; ++i) {
        auto &agent = scene->addChild<Agent>(scene.get());
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

//            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{scale.x(), scale.y(), scale.z()});
            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});
            auto &layoutObject = scene->addChild<RigidBody>(scene.get(), 1.0f, bBoxShape.get(), bWorld);

            layoutObject.scale(scale)
                        .translate({0.5, 0.5, 0.5})
                        .translate({float((bboxMin.x() + bboxMax.x())) / 2, float((bboxMin.y() + bboxMax.y())) / 2, float((bboxMin.z() + bboxMax.z())) / 2});

            // update position of the collision shape
            layoutObject.syncPose();

            layoutObjects.emplace_back(&layoutObject);
            collisionShapes.emplace_back(std::move(bBoxShape));
        }
    }
}

void Env::setAction(int agentIdx, Action action)
{
    currAction[agentIdx] = action;
}

bool Env::step()
{
    static constexpr auto walkSpeed = 0.66f, strafeSpeed = 0.5f;
    static constexpr auto turnSpeed = 7.0_degf;

    std::fill(lastReward.begin(), lastReward.end(), 0.0f);

    for (int i = 0; i < numAgents; ++i) {
        Magnum::Vector3 delta;

        const auto a = currAction[i];
        const auto &agent = agents[i];

        if (!!(a & Action::Forward))
            delta = -walkSpeed * agent->transformation().backward();
        else if (!!(a & Action::Backward))
            delta = walkSpeed * agent->transformation().backward();

        if (!!(a & Action::Left))
            delta = -strafeSpeed * agent->transformation().right();
        else if (!!(a & Action::Right))
            delta = strafeSpeed * agent->transformation().right();

        if (!!(a & Action::LookLeft))
            agent->rotateYLocal(turnSpeed);
        else if (!!(a & Action::LookRight))
            agent->rotateYLocal(-turnSpeed);

        if (agent->allowLookUp) {
            if (!!(a & Action::LookUp))
                agent->rotateXLocal(turnSpeed);
            else if (!!(a & Action::LookDown))
                agent->rotateXLocal(-turnSpeed);
        }

        agent->move(delta, grid);
    }

    bool done = false;
    int numAgentsAtExit = 0;

    for (int i = 0; i < int(agents.size()); ++i) {
        const auto &agent = agents[i];
        const auto t = agent->transformation().translation();

        if (t.x() >= exitPad.min.x() && t.x() <= exitPad.max.x()
            && t.y() >= exitPad.min.y() && t.y() <= exitPad.max.y()
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

    ++episodeDuration;
    if (episodeDuration >= horizon)
        done = true;

    if (episodeDuration % 1000 == 0)
        TLOG(INFO) << "Episode frames " << episodeDuration << "/" << horizon;

    // clear the actions
    for (int i = 0; i < numAgents; ++i)
        currAction[i] = Action::Idle;

    return done;
}
