#include <random>

#include <Magnum/SceneGraph/Camera.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>


using namespace Magnum::Math::Literals;


Env::Env()
{
    TLOG(INFO) << "Creating an environment";

    layoutGenerator.generateFloorWalls(grid);
    layoutGenerator.generateCave(grid);
    layoutDrawables = layoutGenerator.extractPrimitives(grid);

    exitPad = layoutGenerator.levelExit(numAgents);

    auto possibleStartingPositions = layoutGenerator.startingPositions();
    std::shuffle(possibleStartingPositions.begin(), possibleStartingPositions.end(), std::mt19937(std::random_device()()));

    agentStartingPositions = std::vector<VoxelCoords>{possibleStartingPositions.cbegin(), possibleStartingPositions.cbegin() + numAgents};

    for (int i = 0; i < numAgents; ++i)
        agents.emplace_back(std::make_unique<Agent>(&scene));
}

void Env::setAction(int agentIdx, Action action)
{
    currAction[agentIdx] = action;
}

bool Env::step()
{
    constexpr auto turnSpeed = 7.0_degf;

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

    bool done = true;

    for (auto &agent : agents) {
        const auto t = agent->transformation().translation();

        if (t.x() >= exitPad.min.x() && t.x() <= exitPad.max.x()
            && t.y() >= exitPad.min.y() && t.y() <= exitPad.max.y()
            && t.z() >= exitPad.min.z() && t.z() <= exitPad.max.z()) {
            continue;
        }

        done = false;
        break;
    }

    // clear the actions
    for (int i = 0; i < numAgents; ++i)
        currAction[i] = Action::Idle;

    return done;
}
