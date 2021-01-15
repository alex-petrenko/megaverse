#include <scenarios/layout_utils.hpp>
#include <scenarios/scenario_empty.hpp>


using namespace VoxelWorld;


EmptyScenario::EmptyScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
{
}

EmptyScenario::~EmptyScenario() = default;

void EmptyScenario::reset()
{
}

std::vector<Magnum::Vector3> EmptyScenario::agentStartingPositions()
{
    return std::vector<Magnum::Vector3>(env.getNumAgents(), {1, 1, 1});
}

void EmptyScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    addStaticCollidingBox(drawables, envState, {10, 1, 10}, {5, 0, 5}, ColorRgb::BLUE);
}
