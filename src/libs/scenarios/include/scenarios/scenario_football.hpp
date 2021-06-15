#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/layout_utils.hpp>


namespace Megaverse
{

class FootballScenario : public DefaultScenario
{
private:
    class FootballLayout;

public:
    explicit FootballScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~FootballScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int) const override { return 0; }//TODO

    RewardShaping defaultRewardShaping() const override { return {}; }

private:
    VoxelGridComponent<VoxelState> vg;
    PlatformsComponent platformsComponent;

    std::unique_ptr<btSphereShape> collisionShape;
    Object3D *footballObject = nullptr;

    std::unique_ptr<FootballLayout> layout;
};

}