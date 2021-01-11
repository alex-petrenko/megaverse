#include <random>

#include <Magnum/SceneGraph/Camera.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>
#include <env/scenario.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace VoxelWorld;


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
// TODO: make this more flexible, scenario-depenent? Mechanism to disable actions, e.g. when jumping is not needed
const std::vector<int> Env::actionSpaceSizes = {3, 3, 3, 2, 2, 3};


Env::Env(const std::string &scenarioName, int numAgents, FloatParams customFloatParams)
    : scenarioName{scenarioName}
    , state{numAgents}
    , numAgents{numAgents}
{
    scenario = Scenario::create(scenarioName, *this, state);
    scenario->initializeDefaultParameters();
    scenario->setCustomParameters(customFloatParams);

    // empty list of drawables for each supported drawable type
    for (int drawableType = int(DrawableType::First); drawableType < int(DrawableType::NumTypes); ++drawableType)
        drawables[DrawableType(drawableType)] = std::vector<SceneObjectInfo>{};
}

Env::~Env() = default;

void Env::seed(int seedValue)
{
    state.rng.seed((unsigned long)seedValue);
}

void Env::reset()
{
    state.reset();

    auto seed = randRange(0, 1 << 30, state.rng);
    state.rng.seed((unsigned long)seed);
    TLOG(INFO) << "Using seed " << seed;

    // remove dangling pointers from the previous episode
    for (int drawableType = int(DrawableType::First); drawableType < int(DrawableType::NumTypes); ++drawableType)
        drawables[DrawableType(drawableType)].clear();

    scenario->reset();

    scenario->spawnAgents(state.agents);

    scenario->addEpisodeDrawables(drawables);
    scenario->addEpisodeAgentsDrawables(drawables);
}

void Env::setAction(int agentIdx, Action action)
{
    state.currAction[agentIdx] = action;
}

void Env::step()
{
    std::fill(state.lastReward.begin(), state.lastReward.end(), 0.0f);

    const auto lastFrameDurationSec = state.lastFrameDurationSec;

    // TODO: make this more flexible
    for (int i = 0; i < numAgents; ++i) {
        const auto a = state.currAction[i];
        const auto &agent = state.agents[i];

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
    }

    scenario->preStep();

    state.physics.bWorld.stepSimulation(lastFrameDurationSec, 1, state.simulationStepSeconds);

    for (auto agent : state.agents)
        agent->updateTransform();

    scenario->step();

    state.currEpisodeSec += state.lastFrameDurationSec;

    if (state.currEpisodeSec >= episodeLengthSec())
        state.done = true;

    // clear the actions
    for (int i = 0; i < numAgents; ++i)
        state.currAction[i] = Action::Idle;

    for (int i = 0; i < int(state.agents.size()); ++i) {
        state.totalReward[i] += state.lastReward[i];

        if (fabs(state.lastReward[i]) > SIMD_EPSILON)
            TLOG(INFO) << "Last reward for agent #" << i << ":  " << state.lastReward[i] << ", total reward:  " << state.totalReward[i];
    }
}

std::vector<Magnum::Color3> Env::getPalette() const
{
    return scenario->getPalette();
}

float Env::episodeLengthSec() const
{
    return scenario->episodeLengthSec();
}

float Env::trueObjective() const
{
    return scenario->trueObjective();
}
