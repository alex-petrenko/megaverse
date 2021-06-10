#pragma once

#include <memory>


namespace Megaverse
{

class RenderingContext
{
public:
    virtual ~RenderingContext() = default;

    virtual void makeCurrent() = 0;
};


class WindowRenderingContext : public RenderingContext
{
public:
    void makeCurrent() override
    {
        // noop, this is only for test apps that never switch context
    }
};


class WindowlessContext : public RenderingContext
{
public:
    explicit WindowlessContext(int gpuDevice = 0);

    ~WindowlessContext() override;

    void makeCurrent() override;

protected:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}