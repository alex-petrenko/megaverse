#include <Corrade/Containers/Optional.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Timeline.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>
#include <scenarios/init.hpp>
#include <magnum_rendering/windowless_context.hpp>
#include <magnum_rendering/magnum_env_renderer.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


using namespace VoxelWorld;


//const auto scenarioName = "TowerBuilding";
const auto scenarioName = "Football";
//const auto scenarioName = "Obstacles";
//const auto scenarioName = "Collect";
//const auto scenarioName = "Sokoban";
//const auto scenarioName = "BoxAGone";
//const auto scenarioName = "HexMemory";


class Viewer: public Platform::Application
{
public:
    explicit Viewer(const Arguments& arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

    void keyPressEvent(KeyEvent &event) override;
    void keyReleaseEvent(KeyEvent &event) override;
    void mouseMoveEvent(MouseMoveEvent& event) override;

    void handleActions(const KeyEvent::Key &key, bool addAction);
    void controlOverview(const KeyEvent::Key &key, bool addAction);
    void moveOverviewCamera();

private:
    Envs envs;
    std::unique_ptr<MagnumEnvRenderer> renderer;
    std::unique_ptr<RenderingContext> ctx;

    bool withDebugDraw = true;

    int activeEnv = 0, activeAgent = 0;
    Action currAction = Action::Idle;
    Action currOverviewAction = Action::Idle;

    bool forceReset = false;

    Timeline timeline;
};

Viewer::Viewer(const Arguments& arguments):
    Platform::Application{arguments, NoCreate}
{
    scenariosGlobalInit();

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

    const int numAgents = 2;

    auto env = std::make_unique<Env>(scenarioName, numAgents);
//    env->seed(42);
    env->reset();

    const int desiredFps = 15;
    const unsigned int delayMs = 1000 / desiredFps;
    setMinimalLoopPeriod(delayMs);

    env->setSimulationResolution(1.0f / desiredFps);

    envs.emplace_back(std::move(env));

    ctx = std::make_unique<WindowRenderingContext>();

    auto viewport = GL::defaultFramebuffer.viewport();
    renderer = std::make_unique<MagnumEnvRenderer>(envs, viewport.sizeX(), viewport.sizeY(), withDebugDraw, true, ctx.get());
    renderer->reset(*envs[activeEnv], activeEnv);
    renderer->toggleDebugMode();

    timeline.start();
    setSwapInterval(0);
}

void Viewer::drawEvent()
{
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
        renderer->preDraw(*envs[envIdx], envIdx);
        renderer->drawAgent(*envs[envIdx], envIdx, activeAgent, false);
    }

    auto framebuffer = renderer->getFramebuffer();

    GL::defaultFramebuffer.bind();

    framebuffer->mapForRead(GL::Framebuffer::ColorAttachment{0});

    // blit color to window framebuffer
    GL::AbstractFramebuffer::blit(*framebuffer, GL::defaultFramebuffer, {{}, framebuffer->viewport().size()}, GL::FramebufferBlit::Color);

    swapBuffers();

    timeline.nextFrame();
}

void Viewer::tickEvent() {
    auto &env = envs[activeEnv];
    env->setFrameDuration(timeline.previousFrameDuration());
    env->setAction(activeAgent, currAction);

    currAction &= ~Action::Interact;

    env->step();
    const auto done = env->isDone();

    if (done || forceReset) {
        if (done)
            TLOG(INFO) << "Done!";

        std::ostringstream s;
        for (int i = 0; i < env->getNumAgents(); ++i)
            s << " " << env->getTotalReward(i);

        TLOG(INFO) << "Total reward " << s.str();
        TLOG(INFO) << "True objective " << env->trueObjective();
        env->reset();
        renderer->reset(*env, activeEnv);
        forceReset = false;
    }

    moveOverviewCamera();

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

void Viewer::controlOverview(const KeyEvent::Key &key, bool addAction)
{
    auto a = Action::Idle;

    switch(key) {
        case KeyEvent::Key::U: a = Action::Forward; break;
        case KeyEvent::Key::J: a = Action::Backward; break;
        case KeyEvent::Key::H: a = Action::Left; break;
        case KeyEvent::Key::K: a = Action::Right; break;
        default: break;
    }

    if (a == Action::Idle)
        return;

    if (addAction)
        currOverviewAction |= a;
    else
        currOverviewAction &= ~a;

    redraw();
}

void Viewer::moveOverviewCamera()
{
    auto &overview = renderer->getOverview();
    if (!overview.enabled)
        return;

    const auto backwardDirection = overview.verticalTilt->absoluteTransformation().backward();

    if (!!(currOverviewAction & Action::Forward))
        overview.root->translate(-backwardDirection);
    else if (!!(currOverviewAction & Action::Backward))
        overview.root->translate(backwardDirection);

    const auto rightDirection = overview.verticalTilt->absoluteTransformation().right();

    if (!!(currOverviewAction & Action::Left))
        overview.root->translate(-rightDirection);
    else if (!!(currOverviewAction & Action::Right))
        overview.root->translate(rightDirection);

    overview.saveTransformation();
}

void Viewer::keyPressEvent(KeyEvent& event)
{
//    TLOG(INFO) << "Key event " << event.keyName();

    handleActions(event.key(), true);

    controlOverview(event.key(), true);

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
        case KeyEvent::Key::O:
            renderer->getOverview().enabled = !renderer->getOverview().enabled;
            setCursor(renderer->getOverview().enabled ? Cursor::HiddenLocked : Cursor::Arrow);
            break;
        case KeyEvent::Key::Enter:
            renderer->toggleDebugMode();
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
    controlOverview(event.key(), false);
}

void Viewer::mouseMoveEvent(Platform::Sdl2Application::MouseMoveEvent &event)
{
    auto &overview = renderer->getOverview();
    if (!overview.enabled)
        return;

    constexpr float sensitivity = 0.075;
    const auto horizontalMove = float(event.relativePosition().x());
    const auto verticalMove = -float(event.relativePosition().y());

    const auto yRotation = Deg(horizontalMove);
    overview.root->rotateYLocal(-sensitivity * yRotation);

    const auto verticalRotation = sensitivity * verticalMove;
    auto newRotation = overview.verticalRotation + verticalRotation;
    newRotation = std::min(newRotation, 89.0f);
    newRotation = std::max(newRotation, -89.0f);

    const auto rotationDelta = newRotation - overview.verticalRotation;
    const auto xRotation = Deg(rotationDelta);
    overview.verticalTilt->rotateXLocal(xRotation);
    overview.verticalRotation = newRotation;

    overview.saveTransformation();

    event.setAccepted();
    redraw();
}

MAGNUM_APPLICATION_MAIN(Viewer)