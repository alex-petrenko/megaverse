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
#include <magnum_rendering/windowless_context.hpp>
#include <magnum_rendering/magnum_env_renderer.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


class Viewer: public Platform::Application
{
public:
    explicit Viewer(const Arguments& arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

    void keyPressEvent(KeyEvent &event) override;
    void keyReleaseEvent(KeyEvent &event) override;

    void handleActions(const KeyEvent::Key &key, bool addAction);

private:
    std::unique_ptr<Env> env;
    std::unique_ptr<MagnumEnvRenderer> renderer;
    std::unique_ptr<RenderingContext> ctx;

    bool withDebugDraw = true;

    int activeAgent = 0;
    Action currAction = Action::Idle;

    bool forceReset = false;

    Timeline timeline;
};

Viewer::Viewer(const Arguments& arguments):
    Platform::Application{arguments, NoCreate}
{
    // Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x MSAA if we have enough DPI.
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("VoxelEnvViewer").setSize({1280, 720}, dpiScaling);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf)) {
            TLOG(WARNING) << "Fall back to default MSAA";
            create(conf, glConf.setSampleCount(0));
        }
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    const int numAgents = 4;
    const float verticalLookLimitRad = 0.25f;
    env = std::make_unique<Env>(numAgents, verticalLookLimitRad);
    env->setAvailableLayouts({LayoutType::Towers});

    // env->seed(42);
    env->reset();

    ctx = std::make_unique<WindowRenderingContext>();

    auto viewport = GL::defaultFramebuffer.viewport();
    renderer = std::make_unique<MagnumEnvRenderer>(*env, viewport.sizeX(), viewport.sizeY(), withDebugDraw, ctx.get());
    renderer->reset(*env);

    timeline.start();

    const int desiredFps = 15;
    const unsigned int delayMs = 1000 / desiredFps;
    setMinimalLoopPeriod(delayMs);

    env->setSimulationResolution(1.0f / desiredFps);

    setSwapInterval(0);
}


void Viewer::drawEvent()
{
    renderer->drawAgent(*env, activeAgent, false);
    auto framebuffer = renderer->getFramebuffer();

    GL::defaultFramebuffer.bind();

    framebuffer->mapForRead(GL::Framebuffer::ColorAttachment{0});

    // blit color to window framebuffer
    GL::AbstractFramebuffer::blit(*framebuffer, GL::defaultFramebuffer, {{}, framebuffer->viewport().size()}, GL::FramebufferBlit::Color);

    swapBuffers();

    timeline.nextFrame();
}

void Viewer::tickEvent() {
    env->setFrameDuration(timeline.previousFrameDuration());
    env->setAction(activeAgent, currAction);
    currAction &= ~Action::Interact;

    const auto done = env->step();

    if (done || forceReset) {
        if (done)
            TLOG(INFO) << "Done!";

        std::ostringstream s;
        for (int i = 0; i < env->getNumAgents(); ++i)
            s << " " << env->totalReward[i];

        TLOG(INFO) << "Total reward " << s.str();
        TLOG(INFO) << "True objective " << env->trueObjective();
        env->reset();
        renderer->reset(*env);
        forceReset = false;
    }

    redraw();
}

void Viewer::handleActions(const KeyEvent::Key &key, bool addAction)
{
    auto a = Action::Idle;

    switch(key) {
        case KeyEvent::Key::W: a = Action::Forward; break;
        case KeyEvent::Key::S: a = Action::Backward; break;
        case KeyEvent::Key::A: a = Action::Left; break;
        case KeyEvent::Key::D: a = Action::Right; break;

        case KeyEvent::Key::Left: a = Action::LookLeft; break;
        case KeyEvent::Key::Right: a = Action::LookRight; break;
        case KeyEvent::Key::Up: a = Action::LookUp; break;
        case KeyEvent::Key::Down: a = Action::LookDown; break;

        case KeyEvent::Key::Space: a = Action::Jump; break;

        case KeyEvent::Key::E: a = Action::Interact; break;

        default: break;
    }

    if (a == Action::Idle)
        return;

    if (addAction)
        currAction |= a;
    else
        currAction &= ~a;
}


void Viewer::keyPressEvent(KeyEvent& event)
{
//    TLOG(INFO) << "Key event " << event.keyName();

    handleActions(event.key(), true);

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
        case KeyEvent::Key::R:
            forceReset = true;
            break;
        case KeyEvent::Key::Esc:
            exit(0);
        default:
            break;
    }

    event.setAccepted();
}

void Viewer::keyReleaseEvent(Platform::Sdl2Application::KeyEvent &event)
{
    handleActions(event.key(), false);
}

MAGNUM_APPLICATION_MAIN(Viewer)