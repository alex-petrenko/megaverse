#pragma once


#include <env/scenario_component.hpp>


namespace VoxelWorld
{

struct VoxelWithPhysicsObjects : public VoxelState
{
    RigidBody *physicsObject = nullptr;
};

class ObjectStackingCallbacks
{
public:
    virtual bool canPlaceObject(int /*agentIdx*/, const VoxelCoords &, Object3D * /*obj*/) { return true; }
    virtual void placedObject(int /*agentIdx*/, const VoxelCoords &, Object3D * /*obj*/) {}
    virtual void pickedObject(int /*agentIdx*/, const VoxelCoords &, Object3D * /*obj*/) {}
};

/**
 * @tparam VoxelT has to have the field "physicsObject"
 * SFINAE check or C++20 concepts check would be nice here.
 */
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

    void step(Env &env, Env::EnvState &envState) override
    {
        for (int i = 0; i < env.getNumAgents(); ++i) {
            const auto a = envState.currAction[i];
            if (!!(a & Action::Interact))
                onInteractAction(i, envState);
        }
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
            const auto t = obj->absoluteTransformation().translation();

            VoxelCoords voxel = grid.getCoords(t);
            auto voxelPtr = grid.get(voxel);

            bool collidesWithAgent = false;

            for (const auto a : envState.agents) {
                if (a == agent)
                    continue;

                const auto agentTransformation = a->transformation().translation();
                VoxelCoords c = grid.getCoords(agentTransformation);

                if (voxel == c) {
                    collidesWithAgent = true;
                    break;
                }
            }

            const bool empty = !voxelPtr || (voxelPtr->empty() && !voxelPtr->physicsObject);
            if (empty && !collidesWithAgent && callbacks.canPlaceObject(agentIdx, voxel, obj)) {
                // voxel in front of us is empty, can place the object
                // the object should be on the ground or on top of another object
                // descend on y axis until we find ground

                while (true) {
                    VoxelCoords voxelBelow{voxel.x(), voxel.y() - 1, voxel.z()};
                    if (voxelBelow.y() < -30) {
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
                VoxelT voxelState;
                voxelState.physicsObject = obj;
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
            const auto pickup = agent->interactLocation()->absoluteTransformation().translation();
            VoxelCoords voxel = lround(floor(pickup));
            VoxelCoords voxelAbove{voxel.x(), voxel.y() + 1, voxel.z()};

            int pickupHeight = 0, maxPickupHeight = 1;
            while (pickupHeight <= maxPickupHeight) {
                auto voxelPtr = grid.get(voxel), voxelAbovePtr = grid.get(voxelAbove);
                bool hasObjectAbove = voxelAbovePtr && voxelAbovePtr->physicsObject;

                if (voxelPtr && voxelPtr->physicsObject && !hasObjectAbove) {
                    auto obj = voxelPtr->physicsObject;
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

    void addDrawablesAndCollisions(DrawablesMap &drawables, Env::EnvState &envState, const std::vector<VoxelCoords> &objectPositions)
    {
        const auto objSize = 0.39f;
        auto objScale = Magnum::Vector3{objSize, objSize, objSize};

        for (const auto &movableObject : objectPositions) {
            const auto pos = movableObject;
            auto translation = Magnum::Vector3{float(pos.x()) + 0.5f, float(pos.y()) + 0.5f, float(pos.z()) + 0.5f};

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});

            auto &object = envState.scene->addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
            object.scale(objScale).translate(translation);
            object.setCollisionScale({1.15f, 1.15f, 1.15f});
            object.setCollisionOffset({0, -0.05f, 0});
            object.syncPose();

            drawables[DrawableType::Box].emplace_back(&object, rgb(ColorRgb::MOVABLE_BOX));

            envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));

            VoxelT voxelState;
            voxelState.physicsObject = &object;
            grid.set(pos, voxelState);
        }
    }

private:
    VoxelGrid<VoxelT> &grid;

    std::vector<RigidBody *> carryingObject;

    ObjectStackingCallbacks &callbacks;
};

}
