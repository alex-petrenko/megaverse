#pragma once

#include <btBulletDynamicsCommon.h>

#include <Corrade/Containers/Pointer.h>

#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>

#include <util/magnum.hpp>


class RigidBody: public Object3D
{
public:
    RigidBody(Object3D *parent, Magnum::Float mass, btCollisionShape *bShape, btDynamicsWorld &bWorld): Object3D{parent}, bWorld(bWorld)
    {
        /* Calculate inertia so the object reacts as it should with
           rotation and everything */
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        if(!Magnum::Math::TypeTraits<Magnum::Float>::equals(mass, 0.0f))
            bShape->calculateLocalInertia(mass, bInertia);

        // Bullet rigid body setup
        auto* motionState = new Magnum::BulletIntegration::MotionState{*this};  // motion state will update the Object3D transformation
        bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{mass, &motionState->btMotionState(), bShape, bInertia});
        bRigidBody->forceActivationState(DISABLE_DEACTIVATION);  // do we need this?
        bWorld.addRigidBody(bRigidBody.get());
    }

    ~RigidBody() override
    {
        bWorld.removeRigidBody(bRigidBody.get());
    }

    btRigidBody &rigidBody() { return *bRigidBody; }

    /* needed after changing the pose from Magnum side */
    void syncPose()
    {
        bRigidBody->setWorldTransform(btTransform(transformationMatrix()));
    }

private:
    btDynamicsWorld& bWorld;
    Magnum::Containers::Pointer<btRigidBody> bRigidBody;
};