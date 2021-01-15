#include <memory>

#include <map>
#include <mutex>
#include <string>
#include <chrono>

#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>


using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;

using namespace VoxelWorld;


namespace
{

// types
typedef high_resolution_clock::time_point tstamp;

// helper functions
inline microseconds::rep passedSince(const tstamp &tstamp)
{
    const auto now = high_resolution_clock::now();
    const auto passedUsec = duration_cast<microseconds>(now - tstamp).count();
    return passedUsec;
}

}

namespace VoxelWorld
{

struct TinyProfiler::TinyProfilerImpl
{
    std::map<std::string, tstamp> timestamps;
    std::map<std::string, microseconds::rep> totalTime;
};


TinyProfiler & TinyProfiler::instance()
{
    static TinyProfiler profiler;
    return profiler;
}

TinyProfiler & tprof()
{
    return TinyProfiler::instance();
}

}


TinyProfiler::TinyProfiler()
{
    data = std::make_unique<TinyProfilerImpl>();
}

void TinyProfiler::startTimer(const std::string &key)
{
    data->timestamps[key] = high_resolution_clock::now();
}

void TinyProfiler::pauseTimer(const std::string &key)
{
    if (!data->timestamps.count(key))
    {
        TLOG(ERROR) << "No such timer: " << key;
        return;
    }

    const auto passedUsec = passedSince(data->timestamps[key]);
    data->totalTime[key] += passedUsec;
}

float TinyProfiler::readTimer(const std::string &key, bool log)
{
    if (!data->timestamps.count(key))
    {
        TLOG(ERROR) << "No such timer: " << key;
        return 0.0f;
    }
    
    const auto passedUsec = passedSince(data->timestamps[key]) + data->totalTime[key];
    TLOG_IF(INFO, log) << passedUsec << " us passed for " << key;
    return float(passedUsec);
}

float TinyProfiler::stopTimer(const std::string &key, bool log)
{
    const auto passedUsec = readTimer(key, log);
    data->timestamps.erase(key);
    data->totalTime.erase(key);
    return passedUsec;
}
