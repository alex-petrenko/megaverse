#include <scenarios/scenario_football.hpp>


using namespace VoxelWorld;


class FootballScenario::FootballLayout : public EmptyPlatform
{
public:
    FootballLayout(Object3D *parent, Rng &rng, int walls)
    : EmptyPlatform{parent, rng, walls}
    {
    }

    void init() override
    {
        length = randRange(8, 14, rng);
        if (width == -1)
            width = randRange(8, 12, rng);

         height = randRange(3, 7, rng);
    }
};


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

    /**
     * Allows to set collsion shape scale relative to the object scale
     */
    void setCollisionScale(const Magnum::Vector3 &scale)
    {
        collisionScale = scale;
    }

    /* needed after changing the pose from Magnum side */
    void syncPose()
    {
        const auto &m = absoluteTransformationMatrix();
        bRigidBody->setWorldTransform(btTransform{btMatrix3x3{m.rotation()}, btVector3{m.translation()}});
        bRigidBody->getCollisionShape()->setLocalScaling(btVector3{m.scaling() * collisionScale});
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
    Magnum::Vector3 collisionScale{1, 1, 1};
    bool colliding = false;
};


FootballScenario::FootballScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this}
, platformsComponent{*this}
{
}

FootballScenario::~FootballScenario() = default;

void FootballScenario::reset()
{
    vg.reset(env, envState);
    platformsComponent.reset(env, envState);

    collisionShape = std::make_unique<btSphereShape>(2.0);

    auto &object = envState.scene->addChild<DynamicRigidBody>(envState.scene.get(), 1.0f, collisionShape.get(), envState.physics.bWorld);
    auto translation = Magnum::Vector3{5, 5, 5};
    object.scale({0.5, 0.5, 0.5}).translate(translation);
    object.syncPose();

    footballObject = &object;

    layout = std::make_unique<FootballLayout>(platformsComponent.levelRoot.get(), envState.rng, WALLS_ALL);
    layout->init(), layout->generate();
    vg.addPlatform(*layout, true);
}

std::vector<Magnum::Vector3> FootballScenario::agentStartingPositions()
{
    return layout->agentSpawnPoints(env.getNumAgents());
}

void FootballScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();
    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType);

    drawables[DrawableType::Sphere].emplace_back(footballObject, rgb(ColorRgb::ORANGE));
}

void FootballScenario::step()
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
