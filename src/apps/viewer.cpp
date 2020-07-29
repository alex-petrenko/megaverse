#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>

#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Timeline.h>

#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/BulletIntegration/DebugDraw.h>

#include <util/tiny_logger.hpp>
#include <env/env.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


struct InstanceData {
    Matrix4 transformationMatrix;
    Matrix3x3 normalMatrix;
    Color3 color;
};


class CustomDrawable : public SceneGraph::Drawable3D
{
public:
    explicit CustomDrawable(
        SceneGraph::AbstractObject3D &parentObject,
        Containers::Array<InstanceData> &instanceData,
        const Color3 &color,
        const Matrix4 &primitiveTransformation,
        SceneGraph::DrawableGroup3D &drawables
    )
        : SceneGraph::Drawable3D{parentObject, &drawables}, _instanceData(instanceData), _color{color},
        _primitiveTransformation{primitiveTransformation}
    {}

private:
    void draw(const Matrix4& transformation, SceneGraph::Camera3D&) override {
        const Matrix4 t = transformation*_primitiveTransformation;
        arrayAppend(_instanceData, Containers::InPlaceInit, t, t.normalMatrix(), _color);
    }

    Containers::Array<InstanceData>& _instanceData;
    Color3 _color;
    Matrix4 _primitiveTransformation;
};


class SimpleDrawable3D : public Object3D, public SceneGraph::Drawable3D
{
public:
    explicit SimpleDrawable3D(SceneGraph::DrawableGroup3D &drawables,
                              Shaders::Phong & shader, GL::Mesh& mesh, const Color3 &color, Object3D *parentObject)
        : Object3D{parentObject}
          , SceneGraph::Drawable3D{*this, &drawables}
          , _shader(shader), _mesh(mesh), _color(color)
    {}

    void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override
    {
        _shader.setTransformationMatrix(transformationMatrix)
               .setNormalMatrix(transformationMatrix.normalMatrix())
               .setProjectionMatrix(camera.projectionMatrix())
               .setAmbientColor(0.4 * _color)
               .setDiffuseColor(_color)
               .setShininess(150)
                   /* relative to the camera */
               .setLightPosition({13.0f, 2.0f, 5.0f})
               .setLightColor(0xaaaaaa_rgbf)
               .draw(_mesh);
    }

private:
    Shaders::Phong& _shader;
    GL::Mesh& _mesh;
    Color3 _color;
};


class FlatDrawable : public SceneGraph::Drawable3D
{
public:
    explicit FlatDrawable(Object3D &parentObject, SceneGraph::DrawableGroup3D &drawables,
                          Shaders::Flat3D & shader, GL::Mesh& mesh)
        : SceneGraph::Drawable3D{parentObject, &drawables}
        , _shader(shader), _mesh(mesh)
    {}

    void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) override
    {
        _shader.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation).draw(_mesh);
    }

private:
    Shaders::Flat3D& _shader;
    GL::Mesh& _mesh;
};



class Viewer: public Platform::Application
{
public:
    explicit Viewer(const Arguments& arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

    void keyPressEvent(KeyEvent &event) override;

private:
    Env env;
    Action latestAction;

    SceneGraph::DrawableGroup3D drawables;

    GL::Buffer voxelInstanceBuffer{NoCreate};
    Containers::Array<InstanceData> voxelInstanceData;

    Shaders::Phong shader{NoCreate};
    Shaders::Phong shaderInstanced{NoCreate};

    GL::Framebuffer framebuffer;
    GL::Renderbuffer colorBuffer, depthBuffer;

    GL::Mesh cubeMesh, exitPadMesh, agentMesh, agentEyeMesh;

    int activeAgent = 0;
    bool noclipCamera = false;

    Object3D* freeCameraObject;
    SceneGraph::Camera3D* freeCamera;

    btDbvtBroadphase bBroadphase;
    btAlignedObjectArray<btCollisionShape *> collisionShapes;

    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher*	dispatcher;

    btSequentialImpulseConstraintSolver constraintSolver;
    btDiscreteDynamicsWorld *btWorld;

    btPairCachingGhostObject ghostObject;

    btKinematicCharacterController *btCharacter;

    Timeline timeline;
    BulletIntegration::DebugDraw debugDraw{NoCreate};
};

Viewer::Viewer(const Arguments& arguments):
    Platform::Application{arguments, Configuration{}.setTitle("Magnum test")}, framebuffer{GL::defaultFramebuffer.viewport()}
{
    env.reset();

    // bullet
    btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
    collisionShapes.push_back(groundShape);
    collisionConfiguration = new btDefaultCollisionConfiguration();

    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btWorld = new btDiscreteDynamicsWorld(dispatcher, &bBroadphase, &constraintSolver, collisionConfiguration);
//    m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration=0.0001f; ???

    btTransform startTransform;
    startTransform.setIdentity ();
    startTransform.setOrigin (btVector3(0.0, 4.0, 0.0));

    ghostObject.setWorldTransform(startTransform);
    bBroadphase.getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    btScalar characterHeight = 1.75;
    btScalar characterWidth = 0.75;

    btConvexShape* capsule = new btCapsuleShape(characterWidth,characterHeight);
    ghostObject.setCollisionShape (capsule);
    ghostObject.setCollisionFlags (btCollisionObject::CF_CHARACTER_OBJECT);

    auto stepHeight = btScalar(0.35);
    btCharacter = new btKinematicCharacterController (&ghostObject, capsule, stepHeight);
    btCharacter->setGravity(btVector3(0, -9.8, 0));

    ///only collide with static for now (no interaction with dynamic objects)
    auto groundRigidBody = new btRigidBody(0, nullptr, groundShape);
    btWorld->addRigidBody(groundRigidBody);

    btWorld->addCollisionObject(&ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
    btWorld->addAction(btCharacter);


    debugDraw = BulletIntegration::DebugDraw{};
    debugDraw.setMode(BulletIntegration::DebugDraw::Mode::DrawWireframe);
    btWorld->setDebugDrawer(&debugDraw);
    btWorld->setGravity(btVector3(0, -9.8, 0));


    Vector2i framebufferSize{GL::defaultFramebuffer.viewport().sizeX(), GL::defaultFramebuffer.viewport().sizeY()};

    MAGNUM_ASSERT_GL_VERSION_SUPPORTED(GL::Version::GL330);
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::enable(GL::Renderer::Feature::PolygonOffsetFill);
    GL::Renderer::setPolygonOffset(2.0f, 0.5f);

    colorBuffer.setStorage(GL::RenderbufferFormat::SRGB8Alpha8, GL::defaultFramebuffer.viewport().size());
    depthBuffer.setStorage(GL::RenderbufferFormat::DepthComponent24, GL::defaultFramebuffer.viewport().size());

    framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer);
    framebuffer.attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth, depthBuffer);
    framebuffer.mapForDraw({
        {Shaders::Phong::ColorOutput, GL::Framebuffer::ColorAttachment{0}},
    });

    CORRADE_INTERNAL_ASSERT(framebuffer.checkStatus(GL::FramebufferTarget::Draw) == GL::Framebuffer::Status::Complete);

    shaderInstanced = Shaders::Phong{Shaders::Phong::Flag::InstancedTransformation};
    shaderInstanced.setAmbientColor(0x555555_rgbf).setDiffuseColor(0xbbbbbb_rgbf).setShininess(300).setLightPosition({0,4,2}).setLightColor(0xaaaaaa_rgbf);

    shader = Shaders::Phong{};

    // meshes
    {
        agentMesh = MeshTools::compile(Primitives::capsule3DSolid(3, 3, 8, 1.0));
        agentEyeMesh = MeshTools::compile(Primitives::cubeSolid());

        exitPadMesh = MeshTools::compile(Primitives::cubeSolid());

        cubeMesh = MeshTools::compile(Primitives::cubeSolid());
        voxelInstanceBuffer = GL::Buffer{};
        cubeMesh.addVertexBufferInstanced(
            voxelInstanceBuffer, 1, 0,
            Shaders::Phong::TransformationMatrix{},
            Shaders::Phong::NormalMatrix{},
            Shaders::Phong::Color3{}
        );
    }

    // reset renderer data structures
    {
        drawables = SceneGraph::DrawableGroup3D{};
        arrayResize(voxelInstanceData, 0);
    }

    // agents
    {
        const auto &startingPositions = env.agentStartingPositions;
        for (int i = 0; i < env.getNumAgents(); ++i) {
            auto agentPtr = env.agents[i];
            auto pos = Vector3{startingPositions[i]} + Vector3{0.5, 0.525, 0.5};
            agentPtr->rotateY(frand(env.getRng()) * 360.0_degf);
            agentPtr->translate(pos);

            auto & agentDrawable = agentPtr->addChild<SimpleDrawable3D>(drawables, shader, agentMesh, 0xf9d71c_rgbf, agentPtr);
            agentDrawable.scale({0.25f, 0.25f * 0.9f, 0.25f});

            auto & agentEyeDrawable = agentPtr->addChild<SimpleDrawable3D>(drawables, shader, agentEyeMesh, 0x222222_rgbf, agentPtr);
            agentEyeDrawable.scale({0.17, 0.075, 0.17}).translate({0.0f, 0.2f, -0.08f});
        }
    }

    // exit pad
    {
        const auto exitPadCoords = env.exitPad;
        const auto exitPadScale = Vector3(
            exitPadCoords.max.x() - exitPadCoords.min.x(),
            1.0,
            exitPadCoords.max.z() - exitPadCoords.min.z()
        );
        const auto exitPadPos = Vector3(exitPadCoords.min.x() + exitPadScale.x() / 2, exitPadCoords.min.y(),
                                        exitPadCoords.min.z() + exitPadScale.z() / 2);

        auto &exitPadDrawable = env.scene->addChild<SimpleDrawable3D>(drawables, shader, exitPadMesh, 0x50c878_rgbf, env.scene.get());
        exitPadDrawable.scale({0.5, 0.025, 0.5}).scale(exitPadScale);
        exitPadDrawable.translate({0.0, 0.025, 0.0});
        exitPadDrawable.translate(exitPadPos);
    }

    // map layout
    {
        TLOG(INFO) << "Rendering " << env.layoutDrawables.size() << " layout drawables";

        for (auto layoutDrawable : env.layoutDrawables) {
            auto &voxelObject = env.scene->addChild<Object3D>();

            const auto bboxMin = layoutDrawable.min, bboxMax = layoutDrawable.max;
            auto scale = Vector3{
                float(bboxMax.x() - bboxMin.x() + 1) / 2,
                float(bboxMax.y() - bboxMin.y() + 1) / 2,
                float(bboxMax.z() - bboxMin.z() + 1) / 2,
            };

            voxelObject.scale(scale)
                       .translate({0.5, 0.5, 0.5})
                       .translate({float((bboxMin.x() + bboxMax.x())) / 2, float((bboxMin.y() + bboxMax.y())) / 2, float((bboxMin.z() + bboxMax.z())) / 2});

            auto transformation = Matrix4::scaling(Vector3{1.0f});

            voxelObject.addFeature<CustomDrawable>(voxelInstanceData, 0xa5c9ea_rgbf, transformation, drawables);
        }
    }

    /* Configure free camera */
    freeCameraObject = new Object3D{env.scene.get()};
    freeCameraObject->rotateX(0.0_degf);
    freeCameraObject->rotateY(270.0_degf);
    freeCameraObject->translate(Vector3{1.5, 5, 1.5});
    freeCamera = new SceneGraph::Camera3D{*freeCameraObject};
    freeCamera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
              .setProjectionMatrix(Matrix4::perspectiveProjection(75.0_degf, 4.0f/3.0f, 0.1f, 50.0f))
              .setViewport(GL::defaultFramebuffer.viewport().size());

    timeline.start();
}


void Viewer::drawEvent()
{
    framebuffer
        .clearColor(0, Color3{0.125f})
        .clearDepth(1.0f)
        .bind();

    arrayResize(voxelInstanceData, 0);

    auto activeCameraPtr = freeCamera;
    if (!noclipCamera)
        activeCameraPtr = env.agents[activeAgent]->camera;

    activeCameraPtr->draw(drawables);

    shaderInstanced.setProjectionMatrix(activeCameraPtr->projectionMatrix());

    /* Upload instance data to the GPU (orphaning the previous buffer
       contents) and draw all cubes in one call, and all spheres (if any)
       in another call */
    voxelInstanceBuffer.setData(voxelInstanceData, GL::BufferUsage::DynamicDraw);
    cubeMesh.setInstanceCount(voxelInstanceData.size());
    shaderInstanced.draw(cubeMesh);

    //optional but useful: debug drawing
    if (btWorld)
    {
        GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::LessOrEqual);
        debugDraw.setTransformationProjectionMatrix(activeCameraPtr->projectionMatrix()*activeCameraPtr->cameraMatrix());
        btWorld->debugDrawWorld();
        GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::Less);
    }

    /* Bind the main buffer back */
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth).bind();

    /* Blit color to window framebuffer */
    framebuffer.mapForRead(GL::Framebuffer::ColorAttachment{0});
    GL::AbstractFramebuffer::blit(framebuffer, GL::defaultFramebuffer,
                                  {{}, framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    swapBuffers();
    timeline.nextFrame();
}

void Viewer::tickEvent() {
    env.setAction(activeAgent, latestAction);
    const auto done = env.step();

    // bullet
    if (btWorld)
    {
        auto dt = timeline.previousFrameDuration();

        ///set walkDirection for our character
        btTransform xform;
        xform = ghostObject.getWorldTransform ();

        btVector3 forwardDir = xform.getBasis()[2];
        //	printf("forwardDir=%f,%f,%f\n",forwardDir[0],forwardDir[1],forwardDir[2]);
        btVector3 upDir = xform.getBasis()[1];
        btVector3 strafeDir = xform.getBasis()[0];
        forwardDir.normalize ();
        upDir.normalize ();
        strafeDir.normalize ();

        btVector3 walkDirection = btVector3(0.0, 0.0, 0.0);
        auto walkVelocity = btScalar(1.1) * 4.0; // 4 km/h -> 1.1 m/s
        btScalar walkSpeed = walkVelocity * dt;

        //rotate view
        if (!!(latestAction & Action::LookLeft))
        {
            btMatrix3x3 orn = ghostObject.getWorldTransform().getBasis();
            orn *= btMatrix3x3(btQuaternion(btVector3(0,1,0),0.01));
            ghostObject.getWorldTransform ().setBasis(orn);
        }

        if (!!(latestAction & Action::LookRight))
        {
            btMatrix3x3 orn = ghostObject.getWorldTransform().getBasis();
            orn *= btMatrix3x3(btQuaternion(btVector3(0,1,0),-0.01));
            ghostObject.getWorldTransform ().setBasis(orn);
        }

        if (!!(latestAction & Action::Forward))
            walkDirection += forwardDir;

        if (!!(latestAction & Action::Backward))
            walkDirection -= forwardDir;

        btCharacter->setWalkDirection(walkDirection * walkSpeed);

        TLOG(INFO) << "Position: " << xform.getOrigin().x() << " " << xform.getOrigin().y() << " " << xform.getOrigin().z();
        auto gravity = btCharacter->getGravity();
        TLOG(INFO) << "Gravity: " << gravity.x() << " " << gravity.y() << " " << gravity.z();

        int numSimSteps = btWorld->stepSimulation(dt, 5);
//        TLOG(INFO) << "Bt num sim steps: " << numSimSteps;
    }


    latestAction = Action::Idle;

    if (done) {
        TLOG(INFO) << "Done!";
        this->exit(0);
    }

    redraw();
}

void Viewer::keyPressEvent(KeyEvent& event)
{
    switch(event.key()) {
        case KeyEvent::Key::W: latestAction |= Action::Forward; break;
        case KeyEvent::Key::S: latestAction |= Action::Backward; break;
        case KeyEvent::Key::A: latestAction |= Action::Left; break;
        case KeyEvent::Key::D: latestAction |= Action::Right; break;

        case KeyEvent::Key::Left: latestAction |= Action::LookLeft; break;
        case KeyEvent::Key::Right: latestAction |= Action::LookRight; break;
        case KeyEvent::Key::Up: latestAction |= Action::LookUp; break;
        case KeyEvent::Key::Down: latestAction |= Action::LookDown; break;

        default: break;
    }

//    auto objectToMovePtr = freeCameraObject;
//    if (!noclip)
//        objectToMovePtr = env.agents[activeAgent].get();
//
//    Vector3 delta;
//
//    switch(event.key()) {
//        case KeyEvent::Key::N:
//            noclip = !noclip;
//            break;
//        case KeyEvent::Key::C:
//            noclipCamera = !noclipCamera;
//            break;
//
//    }
//
//    if (noclip)
//        freeCameraObject->translate(delta);
//    else
//        moveAgent(*agents[activeAgent], delta, env.getVoxelGrid());
//

    switch (event.key()) {
        case KeyEvent::Key::One:
            activeAgent = 0;
            break;
        case KeyEvent::Key::Two:
            activeAgent = 1;
            break;
        case KeyEvent::Key::Three:
            activeAgent = 2;
            break;
        default:
            break;
    }

    event.setAccepted();
    redraw(); /* camera has changed, redraw! */
}

MAGNUM_APPLICATION_MAIN(Viewer)