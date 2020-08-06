#pragma once

#include <memory>

#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/magnum.hpp>
#include <util/voxel_grid.hpp>

#include <env/physics.hpp>
#include <env/voxel_state.hpp>
#include <env/kinematic_character_controller.hpp>


class Agent : public Object3D
{
public:
    explicit Agent(Object3D *parent, btDynamicsWorld &bWorld, const Magnum::Vector3 &startingPosition, float rotationRad);
    ~Agent() override;

    void updateTransform();

    void lookLeft(float dt);
    void lookRight(float dt);

    btVector3 forwardDirection() const;
    btVector3 strafeLeftDirection() const;

    bool onGround() const;

    void accelerate(const btVector3 &acc, btScalar frameDuration);
    void jump();

private:
    void rotateYAxis(float radians);

public:
    static constexpr auto rotateRadians = 3.0f;

    bool allowLookUp = false;  // TODO?

    Magnum::SceneGraph::Camera3D *camera;

    btDynamicsWorld &bWorld;

    std::unique_ptr<btCapsuleShape> capsuleShape;
    btPairCachingGhostObject ghostObject;
    std::unique_ptr<KinematicCharacterController> bCharacter;
};


