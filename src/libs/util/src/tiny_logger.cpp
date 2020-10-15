#include <mutex>
#include <thread>
#include <cstring>
#include <iomanip>

#include <util/tiny_logger.hpp>
#include <util/filesystem_utils.hpp>


using namespace std::chrono;
using namespace std::chrono_literals;

using namespace VoxelWorld;


namespace
{

/// Everything with level higher than logLevel will be discarded.
LogLevel logLevel = DEBUG;


const char *levelToStr(LogLevel level)
{
    switch (level) {
        case FATAL:
            return "FAT";
        case ERROR:
            return "ERR";
        case WARNING:
            return "WRN";
        case INFO:
            return "INF";
        case VERBOSE:
            return "VER";
        case DEBUG:
            return "DBG";
        default:
            return "";
    }
}

const char *constBasename(const char *filepath)
{
    const char *basename = strrchr(filepath, pathDelim());
    return basename ? basename + 1 : filepath;
}

void printTime(std::ostream &stream)
{
    char timeStr[1 << 6];
    const system_clock::time_point now = system_clock::now();
    const time_t timestamp = system_clock::to_time_t(now);
    strftime(timeStr, sizeof(timeStr), "%m-%d %T", std::localtime(&timestamp));

    const auto sinceEpoch = now.time_since_epoch();
    const system_clock::duration fraction = sinceEpoch - duration_cast<seconds>(sinceEpoch);
    const long long ms = duration_cast<milliseconds>(fraction).count();

    stream << timeStr << '.' << std::setw(3) << std::setfill('0') << ms << ' ';
}

}


namespace VoxelWorld
{

void setLogLevel(LogLevel level)
{
    logLevel = level;
}

LogMessage::LogMessage(LogLevel lvl, const char *file, int line, const char *func, std::ostream *outStream)
    : stream{nullptr}
    , outStream{outStream}
    , level{lvl}
{
    if (level > logLevel) {
        static std::shared_ptr<std::ostringstream> nullStream = std::make_shared<NullStream>();
        stream = nullStream;
    } else {
        stream = std::make_shared<std::ostringstream>();

        *stream << "[";
        printTime(*stream);

        *stream << levelToStr(level) << ' ' << constBasename(file) << ':' << line;
        if (func) {
            *stream << ' ' << func;
            if (!strchr(func, '('))
                *stream << "()";
        }
        *stream << "] ";
    }
}

LogMessage::~LogMessage()
{
    if (stream)
        *stream << '\n';

    if (level <= logLevel) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        *outStream << stream->str();
    }

    if (level == FATAL) {
        *outStream << "\nExiting due to FATAL error, see the logs for details...\n\n\n";
        std::this_thread::sleep_for(2s);
        exit(-1);
    }
}

std::ostringstream &LogMessage::operator()()
{
    return *stream;
}

}