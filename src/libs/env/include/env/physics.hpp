#pragma once

#include <btBulletDynamicsCommon.h>

#include <Corrade/Containers/Pointer.h>

#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>

#include <util/magnum.hpp>

#include <util/tiny_logger.hpp>


namespace VoxelWorld
{

// TODO: refactor this?
class RigidBody : public Object3D
{
public:
    RigidBody(Object3D *parent, Magnum::Float mass, btCollisionShape *bShape, btDynamicsWorld &bWorld)
        : Object3D{parent}, bWorld(bWorld)
    {
        /* Calculate inertia so the object reacts as it should with
           rotation and everything */
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        if (!Magnum::Math::TypeTraits<Magnum::Float>::equals(mass, 0.0f))
            bShape->calculateLocalInertia(mass, bInertia);

        // Bullet rigid body setup
        auto *motionState = new Magnum::BulletIntegration::MotionState{*this};  // motion state will update the Object3D transformation
        bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{mass, &motionState->btMotionState(), bShape, bInertia});
        bRigidBody->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
//        bRigidBody->forceActivationState(DISABLE_DEACTIVATION);  // do we need this?
        bWorld.addRigidBody(bRigidBody.get());
//        auto mask = bRigidBody->getBroadphaseHandle()->m_collisionFilterMask;
        colliding = true;
    }

    ~RigidBody() override
    {
        if (colliding)
            bWorld.removeRigidBody(bRigidBody.get());

        bRigidBody.reset();
    }

    btRigidBody &rigidBody() { return *bRigidBody; }

    void setCollisionOffset(const Magnum::Vector3 &collisionOffsetVec)
    {
        collisionOffset = Magnum::Matrix4::translation(collisionOffsetVec);
    }

    /* needed after changing the pose from Magnum side */
    void syncPose()
    {
        bRigidBody->setWorldTransform(btTransform(absoluteTransformationMatrix()));

//        const auto m = absoluteTransformationMatrix();
//        const auto s = m.scaling();
//        const auto invS = Magnum::Matrix4::scaling({1.0f / s.x(), 1.0f / s.y(), 1.0f / s.z()});
//
//        auto t = collisionOffset * Magnum::Matrix4::translation(m.translation()) * invS *
//                 Magnum::Matrix4::translation(-m.translation()) * m;
//
//        bRigidBody->setWorldTransform(btTransform(t));
    }

    void toggleCollision()
    {
//        bRigidBody->setCollisionFlags(bRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
//        bRigidBody->setActivationState(DISABLE_SIMULATION);

        if (colliding) {
            bWorld.removeRigidBody(bRigidBody.get());
            colliding = false;
        } else {
            bWorld.addRigidBody(bRigidBody.get());
            colliding = true;
        }
    }

private:
    btDynamicsWorld &bWorld;
    Magnum::Containers::Pointer<btRigidBody> bRigidBody;
    Magnum::Matrix4 collisionOffset;
    bool colliding = false;
};

}