#pragma once

#include <env/scenario.hpp>

#include <scenarios/scenario_football.hpp>
#include <scenarios/scenario_obstacles.hpp>
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

    registerScenario<TowerBuilding>("TowerBuilding");
    registerScenario<ObstaclesScenario>("Obstacles");
    registerScenario<FootballScenario>("Football");
}

}