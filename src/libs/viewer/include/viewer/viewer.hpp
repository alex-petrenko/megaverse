#pragma once

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

#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/SceneGraph/Camera.h>

#include <env/env_renderer.hpp>

#include <magnum_rendering/rendering_context.hpp>


namespace Megaverse
{

class Viewer : public Magnum::Platform::Application
{
public:
    explicit Viewer(Envs &envs, bool useVulkan, EnvRenderer *parentRenderer, const Arguments &arguments);

    virtual ~Viewer() { viewerExists = false; }

public:
    void step(const std::vector<bool> &dones);

protected:
    void drawEvent() override;

    void keyPressEvent(KeyEvent &event) override;

    void keyReleaseEvent(KeyEvent &event) override;

    void mouseMoveEvent(MouseMoveEvent &event) override;

private:
    void controlOverview(const KeyEvent::Key &key, bool addAction);

    void moveOverviewCamera();

public:
    static bool viewerExists;

protected:
    Envs &envs;

    /**
     * Overview is only supported for the 1st env in case we have a vector of envs.
     * On-the-fly switching between multiple envs is also possible, but requires additional logic.
     */
    constexpr static int activeEnv = 0;
    int activeAgent = 0;

    std::unique_ptr<EnvRenderer> renderer;

    bool forceReset = false;

private:
    bool useVulkan;

    int width = 1280, height = 720;

    std::unique_ptr<RenderingContext> ctx;

    bool withDebugDraw = true;

    Action currOverviewAction = Action::Idle;

    Magnum::GL::Framebuffer framebuffer{Magnum::NoCreate};
    Magnum::GL::Renderbuffer colorBuffer{Magnum::NoCreate};
};

}
