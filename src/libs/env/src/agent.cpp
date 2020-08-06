#include <memory>

#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <env/agent.hpp>
#include <util/tiny_logger.hpp>
#include <util/util.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


Agent::Agent(Object3D *parent, btDynamicsWorld &bWorld, const Vector3 &startingPosition, float rotationRad)
: Object(parent), bWorld(bWorld)
{
    auto &cameraObject = addChild<Object3D>();
    // cameraObject.rotateY(0.0_degf);
    cameraObject.translate(Magnum::Vector3{0, 0.41f, 0});

    camera = &(cameraObject.addFeature<SceneGraph::Camera3D>());

    camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix4::perspectiveProjection(105.0_degf, 128.0f / 72.0f, 0.1f, 50.0f))
        .setViewport(GL::defaultFramebuffer.viewport().size());

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setRotation(btQuaternion(btVector3(0, 1, 0), rotationRad));
    startTransform.setOrigin (btVector3(startingPosition.x(), startingPosition.y(), startingPosition.z()));
    ghostObject.setWorldTransform(startTransform);

    auto scaleAgents = 1.6f;
    btScalar characterHeight = 0.65f * scaleAgents;
    btScalar characterRadius = 0.22f * scaleAgents;

    capsuleShape = std::make_unique<btCapsuleShape>(characterRadius, characterHeight);

    ghostObject.setCollisionShape(capsuleShape.get());
    ghostObject.setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

    auto stepHeight = btScalar(0.2f);
    bCharacter = std::make_unique<KinematicCharacterController>(&ghostObject, capsuleShape.get(), stepHeight, btVector3(0.0, 1.0, 0.0));

    bWorld.addCollisionObject(&ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
    bWorld.addAction(bCharacter.get());
}

Agent::~Agent()
{
    bWorld.removeCollisionObject(&ghostObject);
    bWorld.removeAction(bCharacter.get());
}

void Agent::updateTransform()
{
    auto worldTrans = ghostObject.getWorldTransform();

    Vector3 position = Vector3{worldTrans.getOrigin()};
    const Vector3 axis = Vector3{worldTrans.getRotation().getAxis()};
    const Float rotation = worldTrans.getRotation().getAngle();

    /* Bullet sometimes reports NaNs for all the parameters and nobody is sure
       why: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12080. The body
       gets stuck in that state, so print the warning just once. */
    if(Math::isNan(position).any() || Math::isNan(axis).any() || Math::isNan(rotation)) {
        Warning{} << "BulletIntegration::MotionState: Bullet reported NaN transform for" << this << Debug::nospace << ", ignoring";
        return;
    }

    position += Vector3{0, 0.05f, 0.0f};

    this->resetTransformation().rotate(Rad{rotation}, axis.normalized()).translate(position);
}

void Agent::lookLeft(float dt)
{
    rotateYAxis(rotateRadians * dt);
}

void Agent::lookRight(float dt)
{
    rotateYAxis(-rotateRadians * dt);
}

void Agent::rotateYAxis(float radians)
{
    btMatrix3x3 orn = ghostObject.getWorldTransform().getBasis();
    orn *= btMatrix3x3(btQuaternion(btVector3(0, 1, 0), radians));
    ghostObject.getWorldTransform ().setBasis(orn);

}

btVector3 Agent::forwardDirection() const
{
    auto xform = ghostObject.getWorldTransform ();
    btVector3 forwardDir = xform.getBasis()[2];
    forwardDir.setZ(-forwardDir.z());
//    TLOG(INFO) << "forwardDir: " << forwardDir.x() << " " << forwardDir.y() << " " << forwardDir.z();
    return forwardDir.normalize();
}

btVector3 Agent::strafeLeftDirection() const
{
    auto xform = ghostObject.getWorldTransform ();
    btVector3 sidewaysDir = xform.getBasis()[0];
    sidewaysDir.setX(-sidewaysDir.x());
    return sidewaysDir.normalize();
}

void Agent::accelerate(const btVector3 &acc, btScalar frameDuration)
{
    bCharacter->setAcceleration(acc, frameDuration);
}

void Agent::jump()
{
    if (onGround())
        bCharacter->jump(btVector3(0, 6.5, 0));
}

bool Agent::onGround() const
{
    return bCharacter->onGround();
}
