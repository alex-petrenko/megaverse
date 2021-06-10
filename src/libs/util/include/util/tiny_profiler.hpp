#pragma once

#include <memory>


namespace Megaverse
{

class TinyProfiler
{
    /// Private implementation to hide some stl headers.
    struct TinyProfilerImpl;

public:
    /// Singleton.
    static TinyProfiler &instance();

    /// Start measurement for key.
    void startTimer(const std::string &key);

    /// Stop time measurement, but keep time measured so far.
    void pauseTimer(const std::string &key);

    /// Read total measured time since start.
    float readTimer(const std::string &key, bool log = false);

    /// Stop measurement for key, returns all measured time since start.
    float stopTimer(const std::string &key, bool log = false);

    TinyProfiler(const TinyProfiler &) = delete;

    void operator=(const TinyProfiler &) = delete;

private:
    TinyProfiler();

private:
    std::unique_ptr<TinyProfilerImpl> data;
};

TinyProfiler &tprof();

}