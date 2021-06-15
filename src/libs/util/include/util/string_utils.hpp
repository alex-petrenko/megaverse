#pragma once

#include <string>
#include <vector>
#include <sstream>


namespace Megaverse
{

std::vector<std::string> splitString(const std::string &s, const std::string &d);

bool startsWith(const std::string &s, const std::string &t);

bool endsWith(const std::string &s, const std::string &t);

std::string toLower(const std::string &s);

template<typename T>
T stringTo(const std::string &s, bool &ok)
{
    T val;
    // slow but easy to write
    if (std::istringstream(s) >> val) {
        ok = true;
        return val;
    }

    ok = false;
    return T();
}

}