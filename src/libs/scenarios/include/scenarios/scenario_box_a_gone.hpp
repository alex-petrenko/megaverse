#pragma once

#include <scenarios/scenario_default.hpp>
#include <scenarios/grid_layout_utils.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_fall_detection.hpp>


namespace VoxelWorld
{

struct VoxelBoxAGone : public VoxelState
{
    Object3D *rewardObject = nullptr;
    int reward = 0;
};


class BoxAGoneScenario : public DefaultScenario, public FallDetectionCallbacks
{
private:
    class BoxAGonePlatform;

public:
    explicit BoxAGoneScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~BoxAGoneScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective() const override { return 0; } // TODO

private:
    VoxelGridComponent<VoxelBoxAGone> vg;
    PlatformsComponent platformsComponent;

    FallDetectionComponent<VoxelBoxAGone> fallDetection;

    std::unique_ptr<BoxAGonePlatform> platform;
};

}