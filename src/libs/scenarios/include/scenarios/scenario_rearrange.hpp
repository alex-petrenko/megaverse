#pragma once

#include <scenarios/layout_utils.hpp>
#include <scenarios/scenario_default.hpp>
#include <scenarios/component_platforms.hpp>
#include <scenarios/component_voxel_grid.hpp>
#include <scenarios/component_object_stacking.hpp>


namespace VoxelWorld
{

struct VoxelRearrange : public VoxelState
{
    RigidBody *physicsObject = nullptr;
};

struct ArrangementItem
{
    ArrangementItem() = default;

    ArrangementItem(DrawableType shape, ColorRgb color, const VoxelCoords &offset)
    : shape{shape}
    , color{color}
    , offset{offset}
    {
    }

    static ArrangementItem random(Rng &rng, const VoxelCoords &offset)
    {
        const static std::vector<DrawableType> shapes{
            DrawableType::Cylinder, DrawableType::Capsule, DrawableType::Box, DrawableType::Sphere,
        };


        const auto shape = randomSample(shapes, rng);
        const auto color = randomObjectColor(rng);

        ArrangementItem item{shape, color, offset};
        return item;
    }

public:
    DrawableType shape = DrawableType::Sphere;
    ColorRgb color = ColorRgb::WHITE;
    VoxelCoords offset{};
};

class ArrangementObject : public RigidBody
{
public:
    ArrangementObject(Object3D *parent, Magnum::Float mass, btCollisionShape *bShape, btDynamicsWorld &bWorld)
    : RigidBody{parent, mass, bShape, bWorld}
    {
    }

public:
    ArrangementItem arrangementItem;
    bool pickedUp = false;
};

struct Arrangement
{
    std::vector<ArrangementItem> items;

    bool contains(DrawableType shape, ColorRgb color, const VoxelCoords &offset) const
    {
        for (const auto &item : items)
            if (item.shape == shape && item.color == color && item.offset == offset)
                return true;

        return false;
    }
};

class RearrangeScenario : public DefaultScenario, public ObjectStackingCallbacks
{
private:
    class RearrangePlatform;

public:
    explicit RearrangeScenario(const std::string &name, Env &env, Env::EnvState &envState);

    ~RearrangeScenario() override;

    // Scenario interface
    void reset() override;

    void step() override;

    std::vector<Magnum::Vector3> agentStartingPositions() override;

    void addEpisodeDrawables(DrawablesMap &drawables) override;

    float trueObjective(int /*agentIdx*/) const override { return solved; }

    RewardShaping defaultRewardShaping() const override
    {
        return {
            {Str::rearrangeOneMoreObjectCorrectPosition, 1},
            {Str::rearrangeAllObjectsCorrectPosition, 10},
        };
    }

    void generateArrangement();

    void arrangementDrawables(DrawablesMap &drawables, const Arrangement &arr, VoxelCoords center, bool interactive);

    int countMatchingObjects() const;

    bool canPlaceObject(int /*agentIdx*/, const VoxelCoords &coords, Object3D *obj) override;

    void placedObject(int agentIdx, const VoxelCoords &coords, Object3D *obj) override;
    void pickedObject(int agentIdx, const VoxelCoords &coords, Object3D *obj) override;

    void checkDone(int agentIdx);

private:
    PlatformsComponent platformsComponent;
    VoxelGridComponent<VoxelRearrange> vg;
    ObjectStackingComponent<VoxelRearrange> objectStackingComponent;

    std::unique_ptr<RearrangePlatform> platform;

    Arrangement arrangement;
    std::vector<ArrangementObject *> arrangementObjects;

    int maxMatchingObjects = 0;

    const VoxelCoords leftCenter = {5, 2, 5};
    const VoxelCoords rightCenter = {13, 2, 5};

    bool solved = false;
};

}