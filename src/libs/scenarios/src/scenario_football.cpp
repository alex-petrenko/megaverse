#include <scenarios/scenario_football.hpp>


using namespace VoxelWorld;


// TODO: refactor this?
class DynamicRigidBody : public Object3D
{
public:
    DynamicRigidBody(Object3D *parent, Magnum::Float mass, btCollisionShape *bShape, btDynamicsWorld &bWorld)
        : Object3D{parent}, bWorld(bWorld)
    {
        /* Calculate inertia so the object reacts as it should with
           rotation and everything */
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        if (!Magnum::Math::TypeTraits<Magnum::Float>::equals(mass, 0.0f))
            bShape->calculateLocalInertia(mass, bInertia);

        // Bullet rigid body setup
        auto *motionState = new Magnum::BulletIntegration::MotionState{*this};  // motion state will update the Object3D transformation
        auto info = btRigidBody::btRigidBodyConstructionInfo{mass, &motionState->btMotionState(), bShape, bInertia};
        info.m_friction = 0.5;
        info.m_rollingFriction = 0.1;
        info.m_spinningFriction = 0.1;

        bRigidBody.emplace(info);
//        bRigidBody->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        bRigidBody->forceActivationState(DISABLE_DEACTIVATION);  // do we need this?
        bWorld.addRigidBody(bRigidBody.get());
//        auto mask = bRigidBody->getBroadphaseHandle()->m_collisionFilterMask;
        colliding = true;
    }

    ~DynamicRigidBody() override
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
        //bRigidBody->setWorldTransform(btTransform(transformationMatrix()));

        const auto m = transformationMatrix();
        const auto s = m.scaling();
        const auto invS = Magnum::Matrix4::scaling({1.0f / s.x(), 1.0f / s.y(), 1.0f / s.z()});

        auto t = collisionOffset * Magnum::Matrix4::translation(m.translation()) * invS *
                 Magnum::Matrix4::translation(-m.translation()) * m;

        bRigidBody->setWorldTransform(btTransform(t));
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

public:
    btDynamicsWorld &bWorld;
    Magnum::Containers::Pointer<btRigidBody> bRigidBody;
    Magnum::Matrix4 collisionOffset;
    bool colliding = false;
};


Football::Football(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this}
, gridLayoutComponent{*this, envState.rng}
{
}

void Football::reset()
{
    vg.reset(env, envState);

    gridLayoutComponent.init(env.getNumAgents(), LayoutType::Empty);
    gridLayoutComponent.generate(vg.grid);

    collisionShape = std::make_unique<btSphereShape>(1.0);

    auto &object = envState.scene->addChild<DynamicRigidBody>(envState.scene.get(), 1.0f, collisionShape.get(), envState.physics.bWorld);
    auto translation = Magnum::Vector3{5, 5, 5};
    object.scale({0.5, 0.5, 0.5}).translate(translation);
    object.syncPose();

    footballObject = &object;
}

std::vector<VoxelCoords> Football::agentStartingPositions()
{
    return gridLayoutComponent.startingPositions(vg.grid);
}

void Football::addEpisodeDrawables(DrawablesMap &drawables)
{
    gridLayoutComponent.addLayoutDrawables(drawables, envState, vg.grid, false);

    drawables[DrawableType::Sphere].emplace_back(footballObject, rgb(ColorRgb::ORANGE));
}

void Football::step()
{
    for (int i = 0; i < env.getNumAgents(); ++i) {
        const auto a = envState.currAction[i];
        if (!!(a & Action::Interact)) {
            const auto &agent = envState.agents[i];
            const auto t = agent->transformation().translation();

            const auto ft = footballObject->transformation().translation();

            auto dt = ft - t;
            if (dt.length() < 1.8) {
                auto dtNorm = dt.normalized();
                dtNorm.y() = 0.5;
                auto force = 70 * btVector3{dtNorm.x(), dtNorm.y(), dtNorm.z()};

                auto football = dynamic_cast<DynamicRigidBody *>(footballObject);
                football->bRigidBody->applyForce(force, {0, 0, 0});
            }
        }
    }
}
