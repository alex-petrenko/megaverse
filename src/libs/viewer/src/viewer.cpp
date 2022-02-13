#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>

#include <viewer/viewer.hpp>

#include <magnum_rendering/magnum_env_renderer.hpp>

#ifndef CORRADE_TARGET_APPLE
    #include <v4r_rendering/v4r_env_renderer.hpp>
#endif


using namespace Magnum;
using namespace Megaverse;


bool Viewer::viewerExists = false;


Viewer::Viewer(Envs &envs, bool useVulkan, EnvRenderer *parentRenderer, const Arguments& arguments)
: Magnum::Platform::Application{arguments, NoCreate}
, envs{envs}
, useVulkan{useVulkan}
{
    assert(!viewerExists);  // only one viewer per process is supported
    viewerExists = true;

    // Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x MSAA if we have enough DPI.
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("MegaverseViewer").setSize({width, height}, dpiScaling);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);

        if(!tryCreate(conf, glConf)) {
            TLOG(WARNING) << "Fall back to default MSAA";
            create(conf, glConf.setSampleCount(0));
        }
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    ctx = std::make_unique<WindowRenderingContext>();

    const Magnum::Vector2i fbSize = framebufferSize();
    if (useVulkan) {
#if defined (CORRADE_TARGET_APPLE)
        TLOG(ERROR) << "Vulkan not supported on MacOS";
        UNUSED(parentRenderer);
#else
        framebuffer = GL::Framebuffer{Range2Di{{}, fbSize}};
        framebuffer.attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer);
        framebuffer.mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}}});
        framebuffer.clearColor(0, Color3{0.125f}).clearDepth(1.0).bind();

        auto v4rParentRenderer = dynamic_cast<V4REnvRenderer *>(parentRenderer);
        renderer = std::make_unique<V4REnvRenderer>(envs, fbSize[0], fbSize[1], v4rParentRenderer, true);
#endif
    } else {
        renderer = std::make_unique<MagnumEnvRenderer>(envs, fbSize[0], fbSize[1], withDebugDraw, true, ctx.get());
        dynamic_cast<MagnumEnvRenderer &>(*renderer).toggleDebugMode();
    }

    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx)
        renderer->reset(*envs[envIdx], envIdx);

    setSwapInterval(0);

    TLOG(WARNING) << "\nControls:\n"
                  << "WASD and arrow keys to control the agent\n"
                  << "1,2,3,4,etc. to switch between agents (if several are present in the environment)\n"
                  << "Press O to toggle the overview camera, use mouse to control view angle\n"
                  << "Use UHJK keys to control the position of the camera\n"
                  << "Press R to reset the episode\n"
                  << "Press ENTER to toggle Bullet collision debug view (only OpenGL version)\n"
                  << "ESC to exit the app\n";
}

void Viewer::step(const std::vector<bool> &dones)
{
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
        if (forceReset)
            envs[envIdx]->terminateEpisodeOnNextFrame();

        if (dones[envIdx])
            renderer->reset(*envs[envIdx], envIdx);
    }

    forceReset = false;

    moveOverviewCamera();
    redraw();
}

void Viewer::drawEvent()
{
    if (useVulkan) {
#if !defined(CORRADE_TARGET_APPLE)
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
               .setStorage(int(Math::log2(height)) + 1, GL::TextureFormat::RGBA8, {width, height})
               .setSubImage(0, {}, image)
               .generateMipmap();

        framebuffer.mapForRead(GL::Framebuffer::ColorAttachment(0));
        framebuffer.attachTexture(GL::Framebuffer::ColorAttachment(0), texture, 0);

        // blit color to window framebuffer
        GL::defaultFramebuffer.bind();
        GL::AbstractFramebuffer::blit(framebuffer, GL::defaultFramebuffer, {{}, framebuffer.viewport().size()}, GL::FramebufferBlit::Color);
#endif
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
}

void Viewer::keyPressEvent(Platform::Sdl2Application::KeyEvent &event)
{
    auto *overview = renderer->getOverview();

    controlOverview(event.key(), true);

    auto chgAgent = [this](int idx) {
        if (idx >= envs[0]->getNumAgents())
            TLOG(WARNING) << "Could not switch to agent " << idx << " (greater than numAgents)";
        else
            activeAgent = idx;
    };

    switch (event.key()) {
        case KeyEvent::Key::One:
            chgAgent(0);
            break;
        case KeyEvent::Key::Two:
            chgAgent(1);
            break;
        case KeyEvent::Key::Three:
            chgAgent(2);
            break;
        case KeyEvent::Key::Four:
            chgAgent(3);
            break;
        case KeyEvent::Key::Five:
            chgAgent(4);
            break;
        case KeyEvent::Key::Six:
            chgAgent(5);
            break;
        case KeyEvent::Key::R:
            forceReset = true;
            break;
        case KeyEvent::Key::O:
            overview->enabled = !overview->enabled;
            setCursor(overview->enabled ? Cursor::HiddenLocked : Cursor::Arrow);
            break;
        case KeyEvent::Key::Enter:
            if (!useVulkan)
                dynamic_cast<MagnumEnvRenderer &>(*renderer).toggleDebugMode();
            break;
        case KeyEvent::Key::Esc:
            setCursor(Cursor::Arrow);
            exit(0);
            std::exit(0);
        default:
            break;
    }

    event.setAccepted();
}

void Viewer::keyReleaseEvent(Platform::Sdl2Application::KeyEvent &event)
{
    controlOverview(event.key(), false);
    event.setAccepted();
}

void Viewer::mouseMoveEvent(Platform::Sdl2Application::MouseMoveEvent &event)
{
    auto *overview = renderer->getOverview();
    if (!overview->enabled)
        return;

    constexpr float sensitivity = 0.075;
    const auto horizontalMove = float(event.relativePosition().x());
    const auto verticalMove = -float(event.relativePosition().y());

    const auto yRotation = Deg(horizontalMove);
    overview->root->rotateYLocal(-sensitivity * yRotation);

    const auto verticalRotation = sensitivity * verticalMove;
    auto newRotation = overview->verticalRotation + verticalRotation;
    newRotation = std::min(newRotation, 89.0f);
    newRotation = std::max(newRotation, -89.0f);

    const auto rotationDelta = newRotation - overview->verticalRotation;
    const auto xRotation = Deg(rotationDelta);
    overview->verticalTilt->rotateXLocal(xRotation);
    overview->verticalRotation = newRotation;

    overview->saveTransformation();

    event.setAccepted();
    redraw();
}

void Viewer::controlOverview(const KeyEvent::Key &key, bool addAction)
{
    auto a = Action::Idle;

    switch(key) {
        case KeyEvent::Key::U: a = Action::Forward; break;
        case KeyEvent::Key::J: a = Action::Backward; break;
        case KeyEvent::Key::H: a = Action::Left; break;
        case KeyEvent::Key::K: a = Action::Right; break;
        case KeyEvent::Key::LeftShift: a = Action::LookUp; break;
        case KeyEvent::Key::RightShift: a = Action::LookDown; break;

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
    auto *overview = renderer->getOverview();
    if (!overview->enabled)
        return;

    const float speed = 0.6;

    Vector3 moveDirection{};

    const auto backwardDirection = overview->verticalTilt->absoluteTransformation().backward();

    if (!!(currOverviewAction & Action::Forward))
        moveDirection -= backwardDirection;
    else if (!!(currOverviewAction & Action::Backward))
        moveDirection += backwardDirection;

    const auto rightDirection = overview->verticalTilt->absoluteTransformation().right();

    if (!!(currOverviewAction & Action::Left))
        moveDirection -= rightDirection;
    else if (!!(currOverviewAction & Action::Right))
        moveDirection += rightDirection;

    const auto upDirection = Vector3{0, 1, 0};

    if (!!(currOverviewAction & Action::LookUp))
        moveDirection += upDirection;
    else if (!!(currOverviewAction & Action::LookDown))
        moveDirection -= upDirection;

    if (moveDirection.length() > FLT_EPSILON)
        overview->root->translate(speed * moveDirection.normalized());

    overview->saveTransformation();
}
