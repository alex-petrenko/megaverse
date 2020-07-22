#pragma once

#include <memory>
#include <vector>
#include <ostream>
#include <sstream>
#include <iostream>


enum LogLevel
{
    FATAL,
    ERROR,
    WARNING,
    INFO,
    VERBOSE,
    DEBUG,
};


void setLogLevel(LogLevel level);


class LogMessage
{
public:
    LogMessage(LogLevel level, const char *file, int line, const char *func, std::ostream *outStream = &std::cout);
    ~LogMessage();

    std::ostringstream & operator()();

private:
    std::shared_ptr<std::ostringstream> stream;
    std::ostream *outStream;
    LogLevel level;
};

class NullStream : public std::ostringstream
{
public:
    NullStream()
        : std::ostringstream()
    {}

    NullStream & operator()()
    {
        return *this;
    }
};

template<typename T>
inline NullStream & operator<<(NullStream &stream, T &&)
{
    return stream;
}


// helper macros

#define TLOG(level) LogMessage(level, __FILE__, __LINE__, __FUNCTION__)()

#define TLOG_IF(level, cond) !(cond) ? NullStream()() : TLOG(level)

// die if condition is not true
#define TCHECK(cond) TLOG_IF(FATAL, !(cond))


// various helper functions

template <typename T>
std::ostream & operator<<(std::ostream &stream, const std::vector<T> &vec)
{
    stream << '[';
    const auto end = std::end(vec);
    for (auto it = std::begin(vec); it != end; ++it)
    {
        stream << *it;
        if (it + 1 != end)
            stream << ", ";
    }
    stream << ']';
    return stream;
}