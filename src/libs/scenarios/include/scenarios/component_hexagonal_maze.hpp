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

    HoneyCombMaze & getMaze() const { return *maze; }

    [[nodiscard]] float getScale() const { return mazeScale; }
    [[nodiscard]] int getSize() const { return mazeSize; }

public:
    int minSize = 2, maxSize = 10;
    float omitWallsProbabilityMin = 0.1f, omitWallsProbabilityMax = 0.7f;

private:
    int mazeSize = 0;
    float mazeScale = 1.0f;
    float wallHeight = 1.0f;
    float omitWallsProbability = 0.0f;
    ColorRgb bottomEdgingColor, topEdgingColor;
    float wallLandmarkProbability = 0.0f;

    std::unique_ptr<HoneyCombMaze> maze;
    double xMin = 0, xMax = 0, yMin = 0, yMax = 0;
};

}