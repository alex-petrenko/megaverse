#pragma once

#include <env/scenario.hpp>

#include <scenarios/scenario_empty.hpp>
#include <scenarios/scenario_sokoban.hpp>
#include <scenarios/scenario_collect.hpp>
#include <scenarios/scenario_football.hpp>
#include <scenarios/scenario_obstacles.hpp>
#include <scenarios/scenario_rearrange.hpp>
#include <scenarios/scenario_hex_memory.hpp>
#include <scenarios/scenario_box_a_gone.hpp>
#include <scenarios/scenario_hex_explore.hpp>
#include <scenarios/scenario_tower_building.hpp>


namespace VoxelWorld
{

template <typename ScenarioType>
void registerScenario(const std::string &name)
{
    Scenario::registerScenario(name, Scenario::scenarioFactory<ScenarioType>);
}

void scenariosGlobalInit()
{
    static bool initialized = false;
    if (initialized)
        return;
    initialized = true;

    // used for debugging and testing
    registerScenario<EmptyScenario>("Empty");

    // experimental
    registerScenario<FootballScenario>("Football");
    registerScenario<BoxAGoneScenario>("BoxAGone");

    // "main" envs
    registerScenario<TowerBuildingScenario>("TowerBuilding");
    registerScenario<ObstaclesEasyScenario>("ObstaclesEasy");
    registerScenario<ObstaclesHardScenario>("ObstaclesHard");
    registerScenario<CollectScenario>("Collect");
    registerScenario<SokobanScenario>("Sokoban");
    registerScenario<HexMemoryScenario>("HexMemory");
    registerScenario<HexExploreScenario>("HexExplore");
    registerScenario<RearrangeScenario>("Rearrange");

    registerScenario<ObstaclesMediumScenario>("ObstaclesMedium");
}

}