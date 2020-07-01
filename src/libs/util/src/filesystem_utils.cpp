#include <util/filesystem_utils.hpp>


size_t readAllBytes(const std::string &filename, std::vector<char> &buffer)
{
    std::ifstream f{ filename, std::ios::in | std::ios::binary };
    return readAllBytes(f, buffer);
}

size_t readAllBytes(std::ifstream &stream, std::vector<char> &buffer)
{
    stream.seekg(0, std::ios::end);
    const auto size = stream.tellg();
    stream.seekg(0);
    buffer.resize(size);
    if (stream.read(buffer.data(), size))
        return size;
    else
        return 0;
}

bool fileExists(const std::string filename)
{
    std::ifstream f(filename);
    return f.good();
}