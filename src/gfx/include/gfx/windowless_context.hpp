#pragma once

#include <memory>


class WindowlessContext
{
public:
    explicit WindowlessContext(int gpuDevice = 0);
    ~WindowlessContext();

    void makeCurrent();

    int gpuDevice() const;

protected:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};
