#include <scenarios/scenario_hex_memory.hpp>

using namespace VoxelWorld;


HexMemoryScenario::HexMemoryScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, maze{*this}
{

}

HexMemoryScenario::~HexMemoryScenario() = default;

void HexMemoryScenario::reset()
{
    maze.reset(env, envState);
}

void HexMemoryScenario::step()
{
    Scenario::step();
}

std::vector<Magnum::Vector3> HexMemoryScenario::agentStartingPositions()
{
    std::vector<Magnum::Vector3> pos(env.getNumAgents(), {0, 2, 0});
    return pos;
}

void HexMemoryScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    maze.addDrawablesAndCollisions(drawables, envState);
}
