#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>


namespace Megaverse
{

inline char pathDelim()
{
#if defined(_WIN32)
    return '\\';
#else
    return '/';
#endif
}

template<typename T>
void pathJoinHelper(std::ostringstream &stream, const T &t)
{
    stream << t;
}

template<typename T, typename... Args>
void pathJoinHelper(std::ostringstream &stream, const T &t, Args... args)
{
    stream << t << pathDelim();
    pathJoinHelper(stream, std::forward<Args>(args)...);
}

template<typename... Args>
std::string pathJoin(Args... args)
{
    std::ostringstream stream;
    pathJoinHelper(stream, std::forward<Args>(args)...);
    return stream.str();
}

/// Returns number of bytes read.
size_t readAllBytes(const std::string &filename, std::vector<char> &buffer);

size_t readAllBytes(std::ifstream &stream, std::vector<char> &buffer);

/// Actually checks if file is accessible.
bool fileExists(const std::string &filename);

std::vector<std::string> listFilesInDirectory(const std::string &dir);

}