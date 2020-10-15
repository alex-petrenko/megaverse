#include <stdio.h>
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btDefaultMotionState.h"

#include <env/kinematic_character_controller.hpp>
#include <util/tiny_logger.hpp>


using namespace VoxelWorld;


// static helper method
static btVector3 getNormalizedVector(const btVector3& v)
{
    btVector3 n(0, 0, 0);

    if (v.length() > SIMD_EPSILON)
    {
        n = v.normalized();
    }
    return n;
}

///@todo Interact with dynamic objects,
///Ride kinematicly animated platforms properly
///More realistic (or maybe just a config option) falling
/// -> Should integrate falling velocity manually and use that in stepDown()
///Support jumping
///Support ducking
class KinematicClosestNotMeRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
public:
    KinematicClosestNotMeRayResultCallback(btCollisionObject* me) : btCollisionWorld::ClosestRayResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
    {
        m_me = me;
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
    {
        if (rayResult.m_collisionObject == m_me)
            return 1.0;

        return ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
    }

protected:
    btCollisionObject* m_me;
};

class KinematicClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
public:
    KinematicClosestNotMeConvexResultCallback(btCollisionObject* me, const btVector3& up, btScalar minSlopeDot)
        : btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0)), m_me(me), m_up(up), m_minSlopeDot(minSlopeDot)
    {
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
    {
        if (convexResult.m_hitCollisionObject == m_me)
            return btScalar(1.0);

        if (!convexResult.m_hitCollisionObject->hasContactResponse())
            return btScalar(1.0);

        btVector3 hitNormalWorld;
        if (normalInWorldSpace)
        {
            hitNormalWorld = convexResult.m_hitNormalLocal;
        }
        else
        {
            ///need to transform normal into worldspace
            hitNormalWorld = convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
        }

        btScalar dotUp = m_up.dot(hitNormalWorld);
        if (dotUp < m_minSlopeDot)
        {
            return btScalar(1.0);
        }

        return ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
    }

protected:
    btCollisionObject* m_me;
    const btVector3 m_up;
    btScalar m_minSlopeDot;
};

/*
 * Returns the reflection direction of a ray going 'direction' hitting a surface with normal 'normal'
 *
 * from: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
 */
btVector3 KinematicCharacterController::computeReflectionDirection(const btVector3& direction, const btVector3& normal)
{
    return direction - (btScalar(2.0) * direction.dot(normal)) * normal;
}

/*
 * Returns the portion of 'direction' that is parallel to 'normal'
 */
btVector3 KinematicCharacterController::parallelComponent(const btVector3& direction, const btVector3& normal)
{
    btScalar magnitude = direction.dot(normal);
    return normal * magnitude;
}

/*
 * Returns the portion of 'direction' that is perpindicular to 'normal'
 */
btVector3 KinematicCharacterController::perpindicularComponent(const btVector3& direction, const btVector3& normal)
{
    return direction - parallelComponent(direction, normal);
}

KinematicCharacterController::KinematicCharacterController(btPairCachingGhostObject* ghostObject, btConvexShape* convexShape, btScalar stepHeight, const btVector3& up)
{
    m_ghostObject = ghostObject;
    m_up.setValue(0.0f, 1.0f, 0.0f);
    m_jumpAxis.setValue(0.0f, 1.0f, 0.0f);
    horizontalVelocity.setValue(0.0, 0.0, 0.0);

    m_AngVel.setValue(0.0, 0.0, 0.0);
    m_useGhostObjectSweepTest = true;
    m_turnAngle = btScalar(0.0);
    m_convexShape = convexShape;
    m_verticalVelocity = 0.0;
    m_verticalOffset = 0.0;
    m_fallSpeed = 55.0;     // Terminal velocity of a sky diver in m/s.
    m_jumpSpeed = 10.0;     // ?
    m_SetjumpSpeed = m_jumpSpeed;
    m_wasOnGround = false;
    m_wasJumping = false;
    m_interpolateUp = true;
    m_currentStepOffset = 0.0;
    m_angularDamping = btScalar(0.0);

    setUp(up);
    setStepHeight(stepHeight);
    setMaxSlope(btRadians(45.0));
}

KinematicCharacterController::~KinematicCharacterController() = default;

btPairCachingGhostObject* KinematicCharacterController::getGhostObject()
{
    return m_ghostObject;
}

bool KinematicCharacterController::recoverFromPenetration(btCollisionWorld* collisionWorld, int iteration)
{
    // Here we must refresh the overlapping paircache as the penetrating movement itself or the
    // previous recovery iteration might have used setWorldTransform and pushed us into an object
    // that is not in the previous cache contents from the last timestep, as will happen if we
    // are pushed into a new AABB overlap. Unhandled this means the next convex sweep gets stuck.
    //
    // Do this by calling the broadphase's setAabb with the moved AABB, this will update the broadphase
    // paircache and the ghostobject's internal paircache at the same time.    /BW

    btVector3 minAabb, maxAabb;
    m_convexShape->getAabb(m_ghostObject->getWorldTransform(), minAabb, maxAabb);
    collisionWorld->getBroadphase()->setAabb(m_ghostObject->getBroadphaseHandle(),
                                             minAabb,
                                             maxAabb,
                                             collisionWorld->getDispatcher());

    bool penetration = false;

    collisionWorld->getDispatcher()->dispatchAllCollisionPairs(m_ghostObject->getOverlappingPairCache(), collisionWorld->getDispatchInfo(), collisionWorld->getDispatcher());

    m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();

    const auto overlappingPairCache = m_ghostObject->getOverlappingPairCache();
    const auto numOverlappingPairs = overlappingPairCache->getNumOverlappingPairs();
    for (int i = 0; i < numOverlappingPairs && !penetration; ++i) {
        m_manifoldArray.resize(0);

        btBroadphasePair* collisionPair = &overlappingPairCache->getOverlappingPairArray()[i];

        auto * obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
        auto * obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

        if ((obj0 && !obj0->hasContactResponse()) || (obj1 && !obj1->hasContactResponse()))
            continue;

        if (!needsCollision(obj0, obj1))
            continue;

        if (collisionPair->m_algorithm)
            collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);

        for (int j = 0; j < m_manifoldArray.size() && !penetration; j++) {
            btPersistentManifold* manifold = m_manifoldArray[j];
            btScalar directionSign = manifold->getBody0() == m_ghostObject ? btScalar(-1.0) : btScalar(1.0);
            for (int p = 0; p < manifold->getNumContacts(); p++) {
                const btManifoldPoint& pt = manifold->getContactPoint(p);

                btScalar dist = pt.getDistance();

                if (dist < -m_maxPenetrationDepth) {
                    auto posDelta = pt.m_normalWorldOnB * directionSign * dist;
                    m_currentPosition += posDelta;
                    penetration = true;
                    break;
                }
            }
        }
    }

    btTransform newTrans = m_ghostObject->getWorldTransform();
    newTrans.setOrigin(m_currentPosition);
    m_ghostObject->setWorldTransform(newTrans);
    //	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
    return penetration;
}

void KinematicCharacterController::stepUp(btCollisionWorld* world)
{
    btScalar stepHeight = 0.0f;
    if (m_verticalVelocity < 0.0)
        stepHeight = m_stepHeight;

    // phase 1: up
    btTransform start, end;

    start.setIdentity();
    end.setIdentity();

    /* FIXME: Handle penetration properly */
    start.setOrigin(m_currentPosition);

    m_targetPosition = m_currentPosition + m_up * (stepHeight) + m_jumpAxis * ((m_verticalOffset > 0.f ? m_verticalOffset : 0.f));
    m_currentPosition = m_targetPosition;

    end.setOrigin(m_targetPosition);

    start.setRotation(m_currentOrientation);
    end.setRotation(m_targetOrientation);

    KinematicClosestNotMeConvexResultCallback callback(m_ghostObject, -m_up, m_maxSlopeCosine);
    callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
    callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

    if (m_useGhostObjectSweepTest)
    {
        m_ghostObject->convexSweepTest(m_convexShape, start, end, callback, world->getDispatchInfo().m_allowedCcdPenetration);
    }
    else
    {
        world->convexSweepTest(m_convexShape, start, end, callback, world->getDispatchInfo().m_allowedCcdPenetration);
    }

    if (callback.hasHit() && m_ghostObject->hasContactResponse() && needsCollision(m_ghostObject, callback.m_hitCollisionObject))
    {
        // Only modify the position if the hit was a slope and not a wall or ceiling.
        if (callback.m_hitNormalWorld.dot(m_up) > 0.0)
        {
            // we moved up only a fraction of the step height
            m_currentStepOffset = stepHeight * callback.m_closestHitFraction;
            if (m_interpolateUp == true)
                m_currentPosition.setInterpolate3(m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
            else
                m_currentPosition = m_targetPosition;
        }

        btTransform& xform = m_ghostObject->getWorldTransform();
        xform.setOrigin(m_currentPosition);
        m_ghostObject->setWorldTransform(xform);

        // fix penetration if we hit a ceiling for example
        int numPenetrationLoops = 0;
        m_touchingContact = false;
        while (recoverFromPenetration(world, numPenetrationLoops))
        {
            numPenetrationLoops++;
            m_touchingContact = true;
            if (numPenetrationLoops > 4)
            {
                //printf("character could not recover from penetration = %d\n", numPenetrationLoops);
                break;
            }
        }
        m_targetPosition = m_ghostObject->getWorldTransform().getOrigin();
        m_currentPosition = m_targetPosition;

        if (m_verticalOffset > 0)
        {
            m_verticalOffset = 0.0;
            m_verticalVelocity = 0.0;
            m_currentStepOffset = m_stepHeight;
        }
    }
    else
    {
        m_currentStepOffset = stepHeight;
        m_currentPosition = m_targetPosition;
    }
}

bool KinematicCharacterController::needsCollision(const btCollisionObject* body0, const btCollisionObject* body1)
{
    bool collides = (body0->getBroadphaseHandle()->m_collisionFilterGroup & body1->getBroadphaseHandle()->m_collisionFilterMask) != 0;
    collides = collides && (body1->getBroadphaseHandle()->m_collisionFilterGroup & body0->getBroadphaseHandle()->m_collisionFilterMask);
    return collides;
}

void KinematicCharacterController::updateTargetPositionBasedOnCollision(const btVector3& hitNormal, btScalar fraction)
{
    btVector3 movementDirection = m_targetPosition - m_currentPosition;
    btScalar movementLength = movementDirection.length();
    if (movementLength > SIMD_EPSILON) {
        movementDirection.normalize();

        auto parallelDir = parallelComponent(movementDirection, hitNormal);
        auto perpindicularDir = perpindicularComponent(movementDirection, hitNormal);

        m_targetPosition = m_currentPosition;

        // arrest the momentum in the direction of the normal, continue movement in the perpendicular direction
        m_targetPosition += perpindicularDir * btScalar(movementLength);
        m_targetPosition += parallelDir * btScalar(movementLength * fraction);
    }
}

/**
 * Phase 2: react to directional user input
 * @param collisionWorld
 * @param horizontalVelocity
 * @param dt
 */
void KinematicCharacterController::stepForwardAndStrafe(btCollisionWorld* collisionWorld, const btVector3& horizontalVelocity, btScalar dt)
{
    m_targetPosition = m_currentPosition + horizontalVelocity * dt;

    btTransform start, end;
    start.setIdentity();
    end.setIdentity();

    int maxIter = 10, iteration = 0;

    while (maxIter-- > 0) {
        ++iteration;

        start.setOrigin(m_currentPosition);
        end.setOrigin(m_targetPosition);
        btVector3 sweepDirNegative(m_currentPosition - m_targetPosition);

        start.setRotation(m_currentOrientation);
        end.setRotation(m_targetOrientation);

        KinematicClosestNotMeConvexResultCallback callback(m_ghostObject, sweepDirNegative, btScalar(0.0));
        callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
        callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

        if (!(start == end)) {
            // desired movement is > 0
            m_ghostObject->convexSweepTest(m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
        }

        if (callback.hasHit() && m_ghostObject->hasContactResponse() && needsCollision(m_ghostObject, callback.m_hitCollisionObject)) {
            // we moved only a fraction
            updateTargetPositionBasedOnCollision(callback.m_hitNormalWorld, callback.m_closestHitFraction);
            btVector3 currentDir = m_targetPosition - m_currentPosition;

            auto distance2 = currentDir.length2();
            if (distance2 > 0.0001f) {
                currentDir.normalize();

                /* See Quake2: "If velocity is against original velocity, stop dead to avoid tiny oscilations in sloping corners." */
                if (currentDir.dot(horizontalVelocity) <= btScalar(0.0)) {
                    // cancel the movement
                    m_targetPosition = m_currentPosition;
                    break;
                }
            } else {
                // cancel the movement
                m_targetPosition = m_currentPosition;
                break;
            }

        } else {
            break;
        }
    }

    m_currentPosition = m_targetPosition;
}


/** phase 3: down
 * @param collisionWorld
 * @param dt
 */
void KinematicCharacterController::stepDown(btCollisionWorld *collisionWorld, btScalar dt)
{
    btScalar downVelocity = (m_verticalVelocity < 0.f ? -m_verticalVelocity : 0.f);

    // limit fall terminal velocity??
    if (downVelocity > 0.0 && downVelocity > m_fallSpeed && (m_wasOnGround || !m_wasJumping))
        downVelocity = m_fallSpeed;

    btVector3 step_drop = m_up * (m_currentStepOffset + downVelocity * dt);
    m_targetPosition -= step_drop;

    KinematicClosestNotMeConvexResultCallback callback(m_ghostObject, m_up, m_maxSlopeCosine);
    callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
    callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

    {
        btTransform start, end;
        start.setIdentity();
        end.setIdentity();

        start.setOrigin(m_currentPosition);
        end.setOrigin(m_targetPosition);
        start.setRotation(m_currentOrientation);
        end.setRotation(m_targetOrientation);

        m_ghostObject->convexSweepTest(m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
    }

    if ((m_ghostObject->hasContactResponse() && (callback.hasHit() && needsCollision(m_ghostObject, callback.m_hitCollisionObject)))) {
        // we dropped a fraction of the height and hit the floor

        // because of allowed penetration this will move us more downwards than we need to (we penetrate the floor polygon)
        // this should be handled later by recoverFromPenetration?
        m_currentPosition.setInterpolate3(m_currentPosition, m_targetPosition, callback.m_closestHitFraction);

        m_verticalVelocity = 0.0;
        m_verticalOffset = 0.0;
        m_wasJumping = false;
    } else {
        // we dropped the full height, didn't hit anything
        m_currentPosition = m_targetPosition;
    }
}


void KinematicCharacterController::setWalkDirection(
    const btVector3& walkDirection)
{
    horizontalVelocity = walkDirection;
}

void KinematicCharacterController::setAngularVelocity(const btVector3& velocity)
{
    m_AngVel = velocity;
}

const btVector3& KinematicCharacterController::getAngularVelocity() const
{
    return m_AngVel;
}

void KinematicCharacterController::setLinearVelocity(const btVector3& velocity)
{
    horizontalVelocity = velocity;

    // HACK: if we are moving in the direction of the up, treat it as a jump :(
    if (horizontalVelocity.length2() > 0)
    {
        btVector3 w = velocity.normalized();
        btScalar c = w.dot(m_up);
        if (c != 0)
        {
            //there is a component in walkdirection for vertical velocity
            btVector3 upComponent = m_up * (btSin(SIMD_HALF_PI - btAcos(c)) * horizontalVelocity.length());
            horizontalVelocity -= upComponent;
            m_verticalVelocity = (c < 0.0f ? -1 : 1) * upComponent.length();

            if (c > 0.0f)
            {
                m_wasJumping = true;
                m_jumpPosition = m_ghostObject->getWorldTransform().getOrigin();
            }
        }
    }
    else
        m_verticalVelocity = 0.0f;
}

btVector3 KinematicCharacterController::getLinearVelocity() const
{
    return horizontalVelocity + (m_verticalVelocity * m_up);
}

void KinematicCharacterController::reset(btCollisionWorld* collisionWorld)
{
    m_verticalVelocity = 0.0;
    m_verticalOffset = 0.0;
    m_wasOnGround = false;
    m_wasJumping = false;
    horizontalVelocity.setValue(0, 0, 0);

    //clear pair cache
    btHashedOverlappingPairCache* cache = m_ghostObject->getOverlappingPairCache();
    while (cache->getOverlappingPairArray().size() > 0)
    {
        cache->removeOverlappingPair(cache->getOverlappingPairArray()[0].m_pProxy0, cache->getOverlappingPairArray()[0].m_pProxy1, collisionWorld->getDispatcher());
    }
}

void KinematicCharacterController::warp(const btVector3& origin)
{
    btTransform xform;
    xform.setIdentity();
    xform.setOrigin(origin);
    m_ghostObject->setWorldTransform(xform);
}

void KinematicCharacterController::preStep(btCollisionWorld* collisionWorld)
{
    m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();
    m_targetPosition = m_currentPosition;

    m_currentOrientation = m_ghostObject->getWorldTransform().getRotation();
    m_targetOrientation = m_currentOrientation;
    //	printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);
}

void KinematicCharacterController::playerStep(btCollisionWorld* collisionWorld, btScalar dt)
{
    const auto originalPosition = m_currentPosition;

    if (m_AngVel.length2() > 0.0f)
        m_AngVel *= btPow(btScalar(1) - m_angularDamping, dt);

    // integrate for angular velocity
    if (m_AngVel.length2() > 0.0f)
    {
        btTransform xform = m_ghostObject->getWorldTransform();

        btQuaternion rot(m_AngVel.normalized(), m_AngVel.length() * dt);

        btQuaternion orn = rot * xform.getRotation();

        xform.setRotation(orn);
        m_ghostObject->setWorldTransform(xform);

        m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();
        m_targetPosition = m_currentPosition;
        m_currentOrientation = m_ghostObject->getWorldTransform().getRotation();
        m_targetOrientation = m_currentOrientation;
    }

    m_wasOnGround = onGround();

    // Update fall velocity.
    m_verticalVelocity -= m_gravity * dt;

    if (m_verticalVelocity > 0.0 && m_verticalVelocity > m_jumpSpeed)
        m_verticalVelocity = m_jumpSpeed;

    if (m_verticalVelocity < 0.0 && btFabs(m_verticalVelocity) > btFabs(m_fallSpeed))
        m_verticalVelocity = -btFabs(m_fallSpeed);

    m_verticalOffset = m_verticalVelocity * dt;

    btTransform xform;
    xform = m_ghostObject->getWorldTransform();

    stepUp(collisionWorld);
    stepForwardAndStrafe(collisionWorld, horizontalVelocity, dt);
    stepDown(collisionWorld, dt);

    xform.setOrigin(m_currentPosition);
    m_ghostObject->setWorldTransform(xform);

    // how far we actually traveled. If we hit a wall this will arrest the momentum
    horizontalVelocity = (m_currentPosition - originalPosition) / dt;
    horizontalVelocity.setY(0);  // we only care about velocity in horizontal plane

    int numPenetrationLoops = 0;
    m_touchingContact = false;
    while (recoverFromPenetration(collisionWorld, numPenetrationLoops))
    {
        numPenetrationLoops++;
        m_touchingContact = true;
        if (numPenetrationLoops > 4)
            break;
    }

    auto currHorizontalSpeed = horizontalVelocity.length();

    if (onGround()) {
        // friction

        if (currHorizontalSpeed - normalDeceleration * dt < 0)
            horizontalVelocity.setZero();
        else
            horizontalVelocity *= (currHorizontalSpeed - normalDeceleration * dt) / currHorizontalSpeed;
    }

//     TLOG(INFO) << "Horizontal speed: " << horizontalVelocity.length();
}

void KinematicCharacterController::setFallSpeed(btScalar fallSpeed)
{
    m_fallSpeed = fallSpeed;
}

void KinematicCharacterController::setJumpSpeed(btScalar jumpSpeed)
{
    m_jumpSpeed = jumpSpeed;
    m_SetjumpSpeed = m_jumpSpeed;
}

void KinematicCharacterController::setMaxJumpHeight(btScalar maxJumpHeight)
{
    m_maxJumpHeight = maxJumpHeight;
}

bool KinematicCharacterController::canJump() const
{
    return onGround();
}

void KinematicCharacterController::jump(const btVector3& v)
{
    m_jumpSpeed = v.length2() == 0 ? m_SetjumpSpeed : v.length();
    m_verticalVelocity = m_jumpSpeed;
    m_wasJumping = true;

    m_jumpAxis = v.length2() == 0 ? m_up : v.normalized();

    m_jumpPosition = m_ghostObject->getWorldTransform().getOrigin();

#if 0
    currently no jumping.
	btTransform xform;
	m_rigidBody->getMotionState()->getWorldTransform (xform);
	btVector3 up = xform.getBasis()[1];
	up.normalize ();
	btScalar magnitude = (btScalar(1.0)/m_rigidBody->getInvMass()) * btScalar(8.0);
	m_rigidBody->applyCentralImpulse (up * magnitude);
#endif
}

void KinematicCharacterController::setGravity(const btVector3& gravity)
{
    if (gravity.length2() > 0) setUpVector(-gravity);

    m_gravity = gravity.length();
}

btVector3 KinematicCharacterController::getGravity() const
{
    return -m_gravity * m_up;
}

void KinematicCharacterController::setMaxSlope(btScalar slopeRadians)
{
    m_maxSlopeRadians = slopeRadians;
    m_maxSlopeCosine = btCos(slopeRadians);
}

btScalar KinematicCharacterController::getMaxSlope() const
{
    return m_maxSlopeRadians;
}

void KinematicCharacterController::setMaxPenetrationDepth(btScalar d)
{
    m_maxPenetrationDepth = d;
}

btScalar KinematicCharacterController::getMaxPenetrationDepth() const
{
    return m_maxPenetrationDepth;
}

bool KinematicCharacterController::onGround() const
{
    return (fabs(m_verticalVelocity) < SIMD_EPSILON) && (fabs(m_verticalOffset) < SIMD_EPSILON);
}

void KinematicCharacterController::setStepHeight(btScalar h)
{
    m_stepHeight = h;
}

btVector3* KinematicCharacterController::getUpAxisDirections()
{
    static btVector3 sUpAxisDirection[3] = {btVector3(1.0f, 0.0f, 0.0f), btVector3(0.0f, 1.0f, 0.0f), btVector3(0.0f, 0.0f, 1.0f)};

    return sUpAxisDirection;
}

void KinematicCharacterController::debugDraw(btIDebugDraw* debugDrawer)
{
}

void KinematicCharacterController::setUpInterpolate(bool value)
{
    m_interpolateUp = value;
}

void KinematicCharacterController::setUp(const btVector3& up)
{
    if (up.length2() > 0 && m_gravity > 0.0f)
    {
        setGravity(-m_gravity * up.normalized());
        return;
    }

    setUpVector(up);
}

void KinematicCharacterController::setUpVector(const btVector3& up)
{
    if (m_up == up)
        return;

    btVector3 u = m_up;

    if (up.length2() > 0)
        m_up = up.normalized();
    else
        m_up = btVector3(0.0, 0.0, 0.0);

    if (!m_ghostObject) return;
    btQuaternion rot = getRotation(m_up, u);

    //set orientation with new up
    btTransform xform;
    xform = m_ghostObject->getWorldTransform();
    btQuaternion orn = rot.inverse() * xform.getRotation();
    xform.setRotation(orn);
    m_ghostObject->setWorldTransform(xform);
}

btQuaternion KinematicCharacterController::getRotation(btVector3& v0, btVector3& v1) const
{
    if (v0.length2() == 0.0f || v1.length2() == 0.0f) {
        btQuaternion q;
        return q;
    }

    return shortestArcQuatNormalize2(v0, v1);
}

/**
 * @param acc acceleration vector
 * @param dt time passed since last frame in seconds
 */
void KinematicCharacterController::setAcceleration(btVector3 acc, btScalar dt)
{
    const bool isOnGround = onGround();

    const auto accelerationMagnitude = acc.length();
    const auto currMaxAcceleration = isOnGround ? maxAcceleration : maxAirAcceleration;

    // we always apply max possible acceleration in the desired direction
    if (!acc.fuzzyZero())
        acc *= currMaxAcceleration / accelerationMagnitude;

    if (isOnGround) {
        // apply acceleration to change velocity
        horizontalVelocity += acc * dt;
        const auto currHorizontalSpeed = horizontalVelocity.length();

        // check if we exceed the max speed, decelerate if necessary
        if (currHorizontalSpeed > maxHorizontalSpeed) {
            const auto dv = exceedingSpeedLimitDeceleration * dt;

            if (currHorizontalSpeed - dv > maxHorizontalSpeed) {
                // after applying large deceleration we're still traveling too fast - just subtract the deceleration
                horizontalVelocity *= (currHorizontalSpeed - dv) / currHorizontalSpeed;
            } else {
                // if we just subtract deceleration we'd be traveling too slow
                // decelerate to max walking speed
                horizontalVelocity *= maxHorizontalSpeed / currHorizontalSpeed;
            }
        }
    } else {
        // in the air

        // there is no friction or deceleration
        const auto currHorizontalSpeed = horizontalVelocity.length();
        const auto newHorizontalVelocity = horizontalVelocity + acc * dt;
        const auto newHorizontalSpeed = newHorizontalVelocity.length();
        if (newHorizontalSpeed <= maxAirSpeed || newHorizontalSpeed < currHorizontalSpeed)
            horizontalVelocity = newHorizontalVelocity;
    }
}
