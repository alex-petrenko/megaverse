#include <memory>

#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <env/agent.hpp>
#include <util/tiny_logger.hpp>
#include <util/util.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace VoxelWorld;


AbstractAgent::AbstractAgent(Object3D *parent, btDynamicsWorld &bWorld, float verticalLookLimitRad)
: Object(parent)
, verticalLookLimitRad{verticalLookLimitRad}
, bWorld(bWorld)
{
}


DefaultKinematicAgent::DefaultKinematicAgent(Object3D *parent, btDynamicsWorld &bWorld, const Vector3 &startingPosition,
                                             float rotationRad, float verticalLookLimitRad)
: AbstractAgent(parent, bWorld, verticalLookLimitRad)
{
    cameraObject = &(addChild<Object3D>());
    // cameraObject.rotateY(0.0_degf);
    cameraObject->translate(Magnum::Vector3{0, 0.41f, 0});

    camera = &(cameraObject->addFeature<SceneGraph::Camera3D>());

    camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix4::perspectiveProjection(110.0_degf, 128.0f / 72.0f, 0.1f, 50.0f))
        .setViewport(GL::defaultFramebuffer.viewport().size());

    pickupSpot = &(cameraObject->addChild<Object3D>());
    pickupSpot->translate({0.0f, -0.44f, -1.0f});

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setRotation(btQuaternion(btVector3(0, 1, 0), rotationRad));
    startTransform.setOrigin (btVector3(startingPosition.x(), startingPosition.y() + getAgentHeight(), startingPosition.z()));
    ghostObject.setWorldTransform(startTransform);

    // btScalar characterHeight = 1.0f;  //1.6
    // btScalar characterRadius = 0.25f;
    // capsuleShape = std::make_unique<btBoxShape>(btVector3{characterRadius, agentHeight / 2, characterRadius});

    btScalar characterHeight = 1.05f;  //1.6
    btScalar characterRadius = 0.33f;
    capsuleShape = std::make_unique<btCapsuleShape>(characterRadius, characterHeight);

    ghostObject.setCollisionShape(capsuleShape.get());
    ghostObject.setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

    auto stepHeight = btScalar(0.2f);
    bCharacter = std::make_unique<KinematicCharacterController>(&ghostObject, capsuleShape.get(), stepHeight, btVector3(0.0, 1.0, 0.0));

//    bWorld.addCollisionObject(&ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::CharacterFilter); // TODO!!!
    bWorld.addCollisionObject(&ghostObject, btBroadphaseProxy::CharacterFilter | btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::CharacterFilter | btBroadphaseProxy::DefaultFilter); // TODO!!!
    bWorld.addAction(bCharacter.get());

    this->updateTransform();
}

DefaultKinematicAgent::~DefaultKinematicAgent()
{
    bWorld.removeCollisionObject(&ghostObject);
    bWorld.removeAction(bCharacter.get());
}

void DefaultKinematicAgent::updateTransform()
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

void DefaultKinematicAgent::lookLeft(float dt)
{
    rotateYAxis(rotateRadians * dt);
}

void DefaultKinematicAgent::lookRight(float dt)
{
    rotateYAxis(-rotateRadians * dt);
}

void DefaultKinematicAgent::lookUp(float dt)
{
    cameraObject->rotateXLocal(Math::Rad<float>(-currXRotation));
    currXRotation += rotateXRadians * dt;
    currXRotation = std::min(verticalLookLimitRad, currXRotation);
    cameraObject->rotateXLocal(Math::Rad<float>(currXRotation));
}

void DefaultKinematicAgent::lookDown(float dt)
{
    cameraObject->rotateXLocal(Math::Rad<float>(-currXRotation));
    // this is a hack, in the beginning of training the agents really love to look at the sky and can't learn anything
    // by making looking down easier than up I hope to prevent this
    currXRotation -= rotateXRadians * dt * 1.1f;
    currXRotation = std::max(-verticalLookLimitRad, currXRotation);
    cameraObject->rotateXLocal(Math::Rad<float>(currXRotation));
}

void DefaultKinematicAgent::rotateYAxis(float radians)
{
    btMatrix3x3 orn = ghostObject.getWorldTransform().getBasis();
    orn *= btMatrix3x3(btQuaternion(btVector3(0, 1, 0), radians));
    ghostObject.getWorldTransform ().setBasis(orn);
}

btVector3 DefaultKinematicAgent::forwardDirection() const
{
    auto xform = ghostObject.getWorldTransform ();
    btVector3 forwardDir = xform.getBasis()[2];
    forwardDir.setZ(-forwardDir.z());
    // TLOG(INFO) << "forwardDir: " << forwardDir.x() << " " << forwardDir.y() << " " << forwardDir.z();
    return forwardDir.normalize();
}

btVector3 DefaultKinematicAgent::strafeLeftDirection() const
{
    auto xform = ghostObject.getWorldTransform ();
    btVector3 sidewaysDir = xform.getBasis()[0];
    sidewaysDir.setX(-sidewaysDir.x());
    return sidewaysDir.normalize();
}

void DefaultKinematicAgent::accelerate(const btVector3 &acc, btScalar frameDuration)
{
    bCharacter->setAcceleration(acc, frameDuration);
}

void DefaultKinematicAgent::jump()
{
    if (onGround())
        bCharacter->jump(btVector3(0, 6.2, 0));
}

bool DefaultKinematicAgent::onGround() const
{
    return bCharacter->onGround();
}
