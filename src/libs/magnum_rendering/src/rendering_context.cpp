// Based on code from:
// https://devblogs.nvidia.com/parallelforall/egl-eye-opengl-visualization-without-x-server/
// https://github.com/facebookresearch/House3D/blob/master/renderer/gl/glContext.cc
// https://github.com/facebookresearch/habitat-sim

#include <fcntl.h>
#include <unistd.h>

#include <Corrade/configure.h>

#include <Magnum/Platform/GLContext.h>
#include <magnum_rendering/rendering_context.hpp>
#include <util/tiny_logger.hpp>

#if defined(CORRADE_TARGET_APPLE)
#include <Magnum/Platform/WindowlessCglApplication.h>
#else
#include <glad/glad_egl.h>
#include <Magnum/Platform/WindowlessEglApplication.h>
#endif

using namespace Megaverse;
namespace Mn = Magnum;

#if !defined(CORRADE_TARGET_APPLE)

constexpr int MAX_DEVICES = 128;

#define CHECK_EGL_ERROR()                                                      \
    do {                                                                       \
        EGLint err = eglGetError();                                            \
        TCHECK(err == EGL_SUCCESS) << "EGL error:" << err;                     \
    } while (0)

bool isNvidiaGpuReadable(int device) {
    const std::string dev = "/dev/nvidia" + std::to_string(device);
    const int retval = open(dev.c_str(), O_RDONLY);
    if (retval == -1)
        return false;
    close(retval);
    return true;
}

struct ContextEGL {
    explicit ContextEGL(int device)
        : magnumGlContext_{Mn::NoCreate} {
        const auto loadStatus = gladLoadEGL();
        TCHECK(loadStatus) << "Failed to load EGL " << loadStatus;

        static const EGLint configAttribs[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE,
            EGL_OPENGL_BIT, EGL_NONE};

        // 1. Initialize EGL
        {
            EGLDeviceEXT eglDevices[MAX_DEVICES];
            EGLint numDevices;
            eglQueryDevicesEXT(MAX_DEVICES, eglDevices, &numDevices);
            CHECK_EGL_ERROR();

            TCHECK(numDevices > 0) << "[EGL] No devices detected";
            TLOG(INFO) << "[EGL] Detected " << numDevices << " EGL devices";

            int eglDevId;
            for (eglDevId = 0; eglDevId < numDevices; ++eglDevId) {
                EGLAttrib cudaDevNumber;

                if (eglQueryDeviceAttribEXT(eglDevices[eglDevId],
                                            EGL_CUDA_DEVICE_NV,
                                            &cudaDevNumber) == EGL_FALSE)
                    continue;

                if (cudaDevNumber == device)
                    break;
            }

            TCHECK(eglDevId < numDevices)
                << "[EGL] Could not find an EGL device for CUDA device "
                << device;

            TCHECK(isNvidiaGpuReadable(eglDevId))
                << "[EGL] EGL device " << eglDevId << ", CUDA device " << device
                << " is not readable";

            TLOG(INFO) << "[EGL] Selected EGL device " << eglDevId
                       << " for CUDA device " << device;
            display_ = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
                                                eglDevices[eglDevId], nullptr);
            CHECK_EGL_ERROR();
        }

        EGLint major, minor;
        EGLBoolean retval = eglInitialize(display_, &major, &minor);
        if (!retval) {
            TLOG(ERROR) << "[EGL] Failed to initialize.";
        }
        CHECK_EGL_ERROR();

        TLOG(INFO) << "[EGL] Version: " << eglQueryString(display_, EGL_VERSION)
                   << " display: " << display_;
        TLOG(INFO) << "[EGL] Vendor: " << eglQueryString(display_, EGL_VENDOR);

        // 2. Select an appropriate configuration
        EGLint numConfigs;
        EGLConfig eglConfig;
        eglChooseConfig(display_, configAttribs, &eglConfig, 1, &numConfigs);
        if (numConfigs != 1) {
            TLOG(ERROR) << "[EGL] Cannot create EGL config. Your driver may "
                           "not support EGL.";
        }
        CHECK_EGL_ERROR();

        // 3. Bind the API
        retval = eglBindAPI(EGL_OPENGL_API);
        if (!retval) {
            TLOG(ERROR) << "[EGL] failed to bind OpenGL API";
        }
        CHECK_EGL_ERROR();

        // 4. Create a context
        context_ = eglCreateContext(display_, eglConfig, EGL_NO_CONTEXT, NULL);
        CHECK_EGL_ERROR();

        // 5. Make context current and create Magnum context
        makeCurrent(false);

        TCHECK(magnumGlContext_.tryCreate())
            << "[EGL] Failed to create OpenGL context";
        isValid_ = true;
    };

    void makeCurrent(bool updateMagnumContext = true) {
        if (Magnum::GL::Context::hasCurrent())
            Magnum::GL::Context::makeCurrent(nullptr);

        EGLBoolean retval =
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_);
        if (!retval)
            TLOG(ERROR) << "[EGL] Failed to make EGL context current";
        CHECK_EGL_ERROR();

        if (updateMagnumContext)
            magnumGlContext_.makeCurrent(&magnumGlContext_);
    };

    bool isValid() { return isValid_; };

    ~ContextEGL() {
        eglDestroyContext(display_, context_);
        //        eglTerminate(display_);

        Magnum::GL::Context::makeCurrent(nullptr);
    }

  private:
    EGLDisplay display_;
    EGLContext context_;
    Mn::Platform::GLContext magnumGlContext_;
    bool isValid_ = false;
};

struct WindowlessContext::Impl {
    explicit Impl(int device) {
        TLOG(INFO);
        glContext = std::make_unique<ContextEGL>(device);
        makeCurrent();
    }

    void makeCurrent() { glContext->makeCurrent(); }

    std::unique_ptr<ContextEGL> glContext{nullptr};
};

#else

struct WindowlessContext::Impl {
    explicit Impl(int device)
        : magnumGLContext_{Mn::NoCreate}, windowlessGLContext_{Mn::NoCreate} {
        TLOG(INFO);
        if (device != 0)
            TLOG(ERROR) << "Context does not support device specification";
        Mn::Platform::WindowlessGLContext::Configuration config;
        windowlessGLContext_ =
            Mn::Platform::WindowlessGLContext{config, &magnumGLContext_};
        if (!windowlessGLContext_.isCreated())
            TLOG(ERROR) << "Unable to create windowless context";
        makeCurrent(false);

        if (!magnumGLContext_.tryCreate())
            TLOG(ERROR) << "Unable to create OpenGL context";
    }

    void makeCurrent(bool updateMagnumContext = true) {
        if (Magnum::GL::Context::hasCurrent())
            Magnum::GL::Context::makeCurrent(nullptr);

        windowlessGLContext_.makeCurrent();

        if (updateMagnumContext)
            magnumGLContext_.makeCurrent(&magnumGLContext_);
    };

    Mn::Platform::GLContext magnumGLContext_;
    Mn::Platform::WindowlessGLContext windowlessGLContext_;
};

#endif

WindowlessContext::WindowlessContext(int device /* = 0 */)
    : pimpl{std::make_unique<WindowlessContext::Impl>(device)} {}

WindowlessContext::~WindowlessContext() = default;

void WindowlessContext::makeCurrent() { pimpl->makeCurrent(); }
