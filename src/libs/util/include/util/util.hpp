#pragma once

#include <random>
#include <cstdint>
#include <algorithm>

#include <util/macro.hpp>


namespace VoxelWorld
{

template<typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template<typename T>
FORCE_INLINE T sqr(T x)
{
    return x * x;
}

using Rng = std::mt19937;

/**
 * @return random integer from [low, high).
 */
inline int randRange(int low, int high, Rng &rng)
{
    return std::uniform_int_distribution<>{low, high - 1}(rng);
}

/**
 * @return random boolean.
 */
inline bool randomBool(Rng &rng)
{
    return bool(randRange(0, 2, rng));
}

/**
 * @return random number in [0, 1)
 */
inline float frand(Rng &rng)
{
    return std::uniform_real_distribution<float>{0, 1}(rng);
}

template<typename CONTAINER_T>
auto randomSample(const CONTAINER_T &container, Rng &rng)
{
    const auto idx = randRange(0, container.size(), rng);
    return container[idx];
}

template<typename T>
void endianSwap(T *x)
{
    int size = sizeof(*x);

    std::reverse(static_cast<uint8_t *>(x), static_cast<uint8_t *>(x + size));
}

template<typename C, typename T>
bool contains(const C &c, const T &val)
{
    return std::find(std::begin(c), std::end(c), val) != std::end(c);
}

void memcpyStride(char *dst, const char *src, int elemSize, int numElements, int ofs, int stride);

}