#pragma once

#include <list>

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_hexagonal_maze.hpp>

namespace VoxelWorld
{

struct CollectableObject
{
    Object3D *object = nullptr;
    bool good = false;
};

struct VoxelHexMemory
{
    std::list<CollectableObject> objects;
};


class HexMemoryScenario : public DefaultScenario
{
public:
    explicit HexMemoryScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~HexMemoryScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    [[nodiscard]] float trueObjective() const override { return 0; }//TODO

private:
    HexagonalMazeComponent maze;

    VoxelGridComponent<VoxelHexMemory> vg;

    Magnum::Vector3 landmarkLocation;
    std::vector<Magnum::Vector3> goodObjects, badObjects;
    int goodObjectsCollected = 0;
};

}