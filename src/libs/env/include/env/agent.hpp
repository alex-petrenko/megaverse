#pragma once

#include <memory>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/magnum.hpp>
#include <util/voxel_grid.hpp>

#include <env/physics.hpp>
#include <env/voxel_state.hpp>
#include <env/kinematic_character_controller.hpp>


namespace VoxelWorld
{

/**
 * TODO: make this interface more compact and flexible
 */
class AbstractAgent : public Object3D
{
public:
    explicit AbstractAgent(Object3D *parent, btDynamicsWorld &bWorld, float verticalLookLimitRad);

    virtual void updateTransform() = 0;

    virtual void lookLeft(float dt) = 0;

    virtual void lookRight(float dt) = 0;

    virtual void lookUp(float dt) = 0;

    virtual void lookDown(float dt) = 0;

    virtual btVector3 forwardDirection() const = 0;

    virtual btVector3 strafeLeftDirection() const = 0;

    virtual bool onGround() const = 0;

    virtual void accelerate(const btVector3 &acc, btScalar frameDuration) = 0;

    virtual void jump() = 0;

    virtual float getAgentHeight() = 0;

    virtual Magnum::SceneGraph::Camera3D * getCamera() = 0;

    virtual Object3D * getCameraObject() = 0;

    virtual Object3D * interactLocation() = 0;

private:
    virtual void rotateYAxis(float radians) = 0;

protected:
    float verticalLookLimitRad = 0.0f;

    btDynamicsWorld &bWorld;
};


class DefaultKinematicAgent : public AbstractAgent
{
public:
    explicit DefaultKinematicAgent(
        Object3D *parent, btDynamicsWorld &bWorld, const Magnum::Vector3 &startingPosition,
        float rotationRad, float verticalLookLimitRad
    );

    ~DefaultKinematicAgent() override;

    void updateTransform() override;

    void lookLeft(float dt) override;

    void lookRight(float dt) override;

    void lookUp(float dt) override;

    void lookDown(float dt) override;

    btVector3 forwardDirection() const override;

    btVector3 strafeLeftDirection() const override;

    bool onGround() const override;

    void accelerate(const btVector3 &acc, btScalar frameDuration) override;

    void jump() override;

    float getAgentHeight() override { return agentHeight; }

    Magnum::SceneGraph::Camera3D * getCamera() override { return camera; }

    Object3D * getCameraObject() override { return cameraObject; }

    Object3D * interactLocation() override { return pickupSpot; }

private:
    void rotateYAxis(float radians) override;

private:
    static constexpr auto rotateRadians = 3.5f, rotateXRadians = 1.5f;
    static constexpr auto agentHeight = 1.75f;

    float currXRotation = 0.0f;

    Magnum::SceneGraph::Camera3D *camera;
    Object3D *cameraObject;
    Object3D *pickupSpot;

    std::unique_ptr<btCapsuleShape> capsuleShape;
    btPairCachingGhostObject ghostObject;
    std::unique_ptr<KinematicCharacterController> bCharacter;
};

}