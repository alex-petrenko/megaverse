#pragma once

#include <env/scenario.hpp>

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

    registerScenario<TowerBuildingScenario>("TowerBuilding");
    registerScenario<ObstaclesEasyScenario>("ObstaclesEasy");
    registerScenario<ObstaclesMediumScenario>("ObstaclesMedium");
    registerScenario<ObstaclesHardScenario>("ObstaclesHard");
    registerScenario<FootballScenario>("Football");
    registerScenario<CollectScenario>("Collect");
    registerScenario<SokobanScenario>("Sokoban");
    registerScenario<BoxAGoneScenario>("BoxAGone");
    registerScenario<HexMemoryScenario>("HexMemory");
    registerScenario<HexExploreScenario>("HexExplore");
    registerScenario<RearrangeScenario>("Rearrange");
}

}