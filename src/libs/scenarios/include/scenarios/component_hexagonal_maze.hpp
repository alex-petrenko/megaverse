#pragma once


#include <env/scenario_component.hpp>


class HoneyCombMaze;

namespace VoxelWorld
{

class HexagonalMazeComponent : public ScenarioComponent
{
public:
    explicit HexagonalMazeComponent(Scenario &scenario);

    ~HexagonalMazeComponent() override;

    void reset(Env &, Env::EnvState &) override;

    void addDrawablesAndCollisions(DrawablesMap &drawables, Env::EnvState &envState) const;

private:
    std::unique_ptr<HoneyCombMaze> maze;
    double xMin = 0, xMax = 0, yMin = 0, yMax = 0;
};

}