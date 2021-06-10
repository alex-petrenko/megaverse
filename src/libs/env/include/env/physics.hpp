#pragma once

#include <btBulletDynamicsCommon.h>

#include <Corrade/Containers/Pointer.h>

#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/SceneGraph/TranslationRotationScalingTransformation3D.h>

#include <util/magnum.hpp>

#include <util/tiny_logger.hpp>


namespace Megaverse
{

class RigidBody : public Object3D
{
public:
    RigidBody(Object3D *parent, Magnum::Float mass, btCollisionShape *bShape, btDynamicsWorld &bWorld)
        : Object3D{parent}, bWorld{bWorld}
    {
        // calculate inertia so the object reacts as it should with rotation and everything
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        if (!Magnum::Math::TypeTraits<Magnum::Float>::equals(mass, 0.0f))
            bShape->calculateLocalInertia(mass, bInertia);

        // bullet rigid body setup
        motionState = std::make_unique<Magnum::BulletIntegration::MotionState>(*this);
        bRigidBody = std::make_unique<btRigidBody>(btRigidBody::btRigidBodyConstructionInfo{mass, &motionState->btMotionState(), bShape, bInertia});

        bRigidBody->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);

        // bRigidBody->forceActivationState(DISABLE_DEACTIVATION);  // do we need this?
        bWorld.addRigidBody(bRigidBody.get());
    }

    ~RigidBody() override
    {
        // similar to Bullet demos, delete motion state first, then remove rigid body from the world, and then delete the rigid body
        motionState.reset();
        bWorld.removeRigidBody(bRigidBody.get());
        bRigidBody.reset();
    }

    btRigidBody &rigidBody() { return *bRigidBody; }

    bool colliding() const
    {
        return !(bRigidBody->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE);
    }

    /**
     * Allows to set collsion shape scale relative to the object scale
     */
    void setCollisionScale(const Magnum::Vector3 &scale)
    {
        collisionScale = scale;
    }

    void setCollisionOffset(const Magnum::Vector3 &offset)
    {
        collisionOffset = offset;
    }

    /* needed after changing the pose from Magnum side */
    void syncPose()
    {
        const auto &m = absoluteTransformationMatrix();
        bRigidBody->setWorldTransform(btTransform{btMatrix3x3{m.rotation()}, btVector3{m.translation() + collisionOffset}});
        bRigidBody->getCollisionShape()->setLocalScaling(btVector3{m.scaling() * collisionScale});
    }

    void toggleCollision()
    {
        const auto flags = bRigidBody->getCollisionFlags();
        if (flags & btCollisionObject::CF_NO_CONTACT_RESPONSE) {
            // no collisions for this object, enabling collisions...
            bRigidBody->setCollisionFlags(flags & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
        } else {
            bRigidBody->setCollisionFlags(flags | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        }
    }

private:
    btDynamicsWorld &bWorld;
    std::unique_ptr<btRigidBody> bRigidBody;
    std::unique_ptr<Magnum::BulletIntegration::MotionState> motionState;
    Magnum::Vector3 collisionScale{1, 1, 1};
    Magnum::Vector3 collisionOffset;
};

}