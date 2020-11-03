#pragma once


#include <env/scenario_component.hpp>


namespace VoxelWorld
{

class ObjectStackingCallbacks
{
public:
    virtual bool canPlaceObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) = 0;
    virtual void placedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) = 0;
    virtual void pickedObject(int agentIdx, const VoxelCoords &voxel, Object3D *obj) = 0;
};

template<typename VoxelT>
class ObjectStackingComponent : public ScenarioComponent
{
public:
    explicit ObjectStackingComponent(Scenario &scenario, int numAgents, VoxelGrid<VoxelT> &grid, ObjectStackingCallbacks &callbacks)
    : ScenarioComponent{scenario}
    , grid{grid}
    , carryingObject(size_t(numAgents), nullptr)
    , callbacks{callbacks}
    {
    }

    void reset(Env &, Env::EnvState &) override
    {
        std::fill(carryingObject.begin(), carryingObject.end(), nullptr);
    }

    Object3D * agentCarryingObject(int agentIdx) const
    {
        return carryingObject[agentIdx];
    }

    void onInteractAction(int agentIdx, Env::EnvState &envState)
    {
        AbstractAgent *agent = envState.agents[agentIdx];

        const auto carryingScale = 0.78f, carryingScaleInverse = 1.0f / carryingScale;

        // putting object on the ground
        if (carryingObject[agentIdx]) {
            auto obj = carryingObject[agentIdx];
            auto t = obj->absoluteTransformation().translation();

            VoxelCoords voxel{t};
            auto voxelPtr = grid.get(voxel);

            bool collidesWithAgent = false;

            for (const auto a : envState.agents) {
                if (a == agent)
                    continue;

                const auto agentTransformation = a->transformation().translation();
                VoxelCoords c{agentTransformation};

                if (voxel == c) {
                    collidesWithAgent = true;
                    break;
                }
            }

            if (!voxelPtr && !collidesWithAgent && callbacks.canPlaceObject(agentIdx, voxel, obj)) {
                // voxel in front of us is empty, can place the object
                // the object should be on the ground or on top of another object
                // descend on y axis until we find ground

                while (true) {
                    VoxelCoords voxelBelow{voxel.x(), voxel.y() - 1, voxel.z()};
                    if (voxelBelow.y() < 0) {
                        // this is the lowest level we support
                        break;
                    }

                    auto voxelBelowPtr = grid.get(voxelBelow);
                    if (voxelBelowPtr)
                        break;
                    else
                        voxel = voxelBelow;
                }

                // placing object on the ground (or another object)
                VoxelT voxelState{false};
                voxelState.obj = obj;
                grid.set(voxel, voxelState);

                obj->setParent(envState.scene.get());

                auto scaling = obj->transformation().scaling();
                obj->resetTransformation();
                obj->scale({scaling.x() * carryingScaleInverse, scaling.y() * carryingScaleInverse, scaling.z() * carryingScaleInverse});
                obj->translate({float(voxel.x()) + 0.5f, float(voxel.y()) + 0.5f, float(voxel.z()) + 0.5f});
                obj->syncPose();

                obj->toggleCollision();

                carryingObject[agentIdx] = nullptr;

                callbacks.placedObject(agentIdx, voxel, obj);
            }

        } else {
            // picking up an object
            const auto &pickup = agent->interactLocation()->absoluteTransformation().translation();
            VoxelCoords voxel{int(pickup.x()), int(pickup.y()), int(pickup.z())};
            VoxelCoords voxelAbove{voxel.x(), voxel.y() + 1, voxel.z()};

            int pickupHeight = 0, maxPickupHeight = 1;
            while (pickupHeight <= maxPickupHeight) {
                auto voxelPtr = grid.get(voxel), voxelAbovePtr = grid.get(voxelAbove);
                bool hasObjectAbove = voxelAbovePtr && voxelAbovePtr->obj;

                if (voxelPtr && voxelPtr->obj && !hasObjectAbove) {
                    auto obj = voxelPtr->obj;
                    obj->toggleCollision();

                    obj->setParent(agent);
                    auto scaling = obj->transformation().scaling();
                    obj->resetTransformation();

                    obj->scale({scaling.x() * carryingScale, scaling.y() * carryingScale, scaling.z() * carryingScale});
                    obj->translate({0.0f, -0.3f, 0.0f});
                    obj->setParent(agent->interactLocation());

                    carryingObject[agentIdx] = obj;

                    grid.remove(voxel);

                    callbacks.pickedObject(agentIdx, voxel, obj);

                    break;

                } else {
                    voxel = voxelAbove;
                    voxelAbove = VoxelCoords{voxel.x(), voxel.y() + 1, voxel.z()};
                }

                ++pickupHeight;
            }
        }
    }

private:
    VoxelGrid<VoxelT> &grid;

    std::vector<RigidBody *> carryingObject;

    ObjectStackingCallbacks &callbacks;
};

}