#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <Magnum/Timeline.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/TextureFormat.h>

#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/SceneGraph/Camera.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>
#include <scenarios/init.hpp>

#include <magnum_rendering/windowless_context.hpp>
#include <magnum_rendering/magnum_env_renderer.hpp>

#include <v4r_rendering/v4r_env_renderer.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


using namespace VoxelWorld;

// TODO: CLI parameters
const bool useVulkan = false;

// "main" envs
//const auto scenarioName = "ObstaclesHard";  // *
//const auto scenarioName = "ObstaclesEasy";  // *
const auto scenarioName = "Collect";    // *
//const auto scenarioName = "Sokoban";  // *
//const auto scenarioName = "TowerBuilding";
//const auto scenarioName = "HexMemory";  // *
//const auto scenarioName = "HexExplore";  // *
//const auto scenarioName = "Rearrange";  // *

// test and experimental envs
//const auto scenarioName = "Test";
//const auto scenarioName = "Empty";
//const auto scenarioName = "BoxAGone";
//const auto scenarioName = "Football";
//const auto scenarioName = "ObstaclesWalls";
//const auto scenarioName = "ObstaclesSteps";
//const auto scenarioName = "ObstaclesLava";


class Viewer: public Magnum::Platform::Application
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
    int width = 1800, height = 1000;

    Envs envs;
    std::unique_ptr<EnvRenderer> renderer;
    std::unique_ptr<RenderingContext> ctx;

    bool withDebugDraw = true;

    int activeEnv = 0, activeAgent = 0;
    Action currAction = Action::Idle;
    Action currOverviewAction = Action::Idle;

    bool forceReset = false;

    Timeline timeline;

    GL::Framebuffer framebuffer{NoCreate};
    GL::Renderbuffer colorBuffer{NoCreate};
};

Viewer::Viewer(const Arguments& arguments)
: Magnum::Platform::Application{arguments, NoCreate}
{
    scenariosGlobalInit();

    // Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x MSAA if we have enough DPI.
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("VoxelEnvViewer").setSize({width, height}, dpiScaling);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf)) {
            TLOG(WARNING) << "Fall back to default MSAA";
            create(conf, glConf.setSampleCount(0));
        }
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    const int numAgents = 3;

    FloatParams params{{Str::useUIRewardIndicators, 1.0f}};
//    FloatParams params{};
    auto env = std::make_unique<Env>(scenarioName, numAgents, params);
//    env->seed(42);
    env->reset();

    const int desiredFps = 15;
    const unsigned int delayMs = 1000 / desiredFps;
    setMinimalLoopPeriod(delayMs);

    env->setSimulationResolution(1.0f / desiredFps);

    envs.emplace_back(std::move(env));

    ctx = std::make_unique<WindowRenderingContext>();

    if (useVulkan) {
        framebuffer = GL::Framebuffer{Range2Di{{}, Vector2i{width, height}}};
        framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer);
        framebuffer.mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}}});
        framebuffer.clearColor(0, Color3{0.125f}).clearDepth(1.0).bind();

        renderer = std::make_unique<V4REnvRenderer>(envs, width, height, nullptr);
    } else {
        renderer = std::make_unique<MagnumEnvRenderer>(envs, width, height, withDebugDraw, true, ctx.get());
        dynamic_cast<MagnumEnvRenderer &>(*renderer).toggleDebugMode();
    }

    renderer->reset(*envs[activeEnv], activeEnv);

    timeline.start();
    setSwapInterval(0);
}

void Viewer::drawEvent()
{
    if (useVulkan) {
        for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx)
            renderer->preDraw(*envs[envIdx], envIdx);

        renderer->draw(envs);
        auto dataPtr = renderer->getObservation(activeEnv, activeAgent);

        // probably would've been easier to actually draw a quad upside down instead of flipping the texture
        // but hey, this is not a performance-critical code and it works!
        cv::Mat mat(height, width, CV_8UC4, (char *) dataPtr);
        cv::flip(mat, mat, 0);

        Containers::ArrayView<const uint8_t> data(dataPtr, width * height * 4);
        ImageView2D image(PixelFormat::RGBA8Unorm, {width, height}, data);

        GL::Texture2D texture;
        texture.setWrapping(GL::SamplerWrapping::ClampToEdge)
               .setStorage(Math::log2(height) + 1, GL::TextureFormat::RGBA8, {width, height})
               .setSubImage(0, {}, image)
               .generateMipmap();

        framebuffer.mapForRead(GL::Framebuffer::ColorAttachment(0));
        framebuffer.attachTexture(GL::Framebuffer::ColorAttachment(0), texture, 0);

        // blit color to window framebuffer
        GL::defaultFramebuffer.bind();
        GL::AbstractFramebuffer::blit(framebuffer, GL::defaultFramebuffer, {{}, framebuffer.viewport().size()}, GL::FramebufferBlit::Color);

    } else {
        auto &magnumRenderer = dynamic_cast<MagnumEnvRenderer &>(*renderer);

        for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
            magnumRenderer.preDraw(*envs[envIdx], envIdx);
            magnumRenderer.drawAgent(*envs[envIdx], envIdx, activeAgent, false);
        }

        auto rendererFramebuffer = magnumRenderer.getFramebuffer();

        GL::defaultFramebuffer.bind();

        rendererFramebuffer->mapForRead(GL::Framebuffer::ColorAttachment{0});

        // blit color to window framebuffer
        GL::AbstractFramebuffer::blit(*rendererFramebuffer, GL::defaultFramebuffer, {{}, rendererFramebuffer->viewport().size()}, GL::FramebufferBlit::Color);
    }

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
        for (int i = 0; i < env->getNumAgents(); ++i) s << " " << env->getTotalReward(i);
        TLOG(INFO) << "Total reward: " << s.str();
        s.clear();

        for (int i = 0; i < env->getNumAgents(); ++i) s << " " << env->trueObjective(i);
        TLOG(INFO) << "True objectives: " << s.str();

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
    if (useVulkan)
        return;

    auto &overview = dynamic_cast<MagnumEnvRenderer &>(*renderer).getOverview();
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

    Overview *overview;
    if (!useVulkan)
        overview = &(dynamic_cast<MagnumEnvRenderer &>(*renderer).getOverview());  // TODO

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
            if (!useVulkan) {
                overview->enabled = !overview->enabled;
                setCursor(overview->enabled ? Cursor::HiddenLocked : Cursor::Arrow);
            }
            break;
        case KeyEvent::Key::Enter:
            if (!useVulkan)
                dynamic_cast<MagnumEnvRenderer &>(*renderer).toggleDebugMode();
            break;
        case KeyEvent::Key::Esc:
            exit(0);
        default:
            break;
    }

    event.setAccepted();
}

void Viewer::keyReleaseEvent(Magnum::Platform::Sdl2Application::KeyEvent &event)
{
    handleActions(event.key(), false);
    controlOverview(event.key(), false);
}

void Viewer::mouseMoveEvent(Magnum::Platform::Sdl2Application::MouseMoveEvent &event)
{
    if (useVulkan)
        return;

    auto &overview = dynamic_cast<MagnumEnvRenderer &>(*renderer).getOverview();
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