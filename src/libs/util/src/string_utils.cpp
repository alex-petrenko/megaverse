#include <cstring>

#include <util/string_utils.hpp>


namespace VoxelWorld
{

std::vector<std::string> splitString(const std::string &s, const std::string &d)
{
    std::vector<std::string> result;

    char *cStr = new char[s.size() + 1];
    strcpy(cStr, s.c_str());
    char *strtokPtr = nullptr;
    char *token = strtok_r(cStr, d.c_str(), &strtokPtr);
    while (token) {
        result.emplace_back(token);
        token = strtok_r(nullptr, d.c_str(), &strtokPtr);
    }

    delete[] cStr;
    return result;
}

bool startsWith(const std::string &s, const std::string &t)
{
    if (t.empty())
        return true;
    return s.find(t) == 0;
}

bool endsWith(const std::string &s, const std::string &t)
{
    if (t.size() > s.size())
        return false;
    if (t.empty())
        return true;
    return s.find(t) == s.size() - t.size();
}

}