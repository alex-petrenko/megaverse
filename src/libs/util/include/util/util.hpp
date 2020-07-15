#pragma once

#include <cstdint>
#include <algorithm>

#include <util/macro.hpp>


#define ARR_LENGTH(arr) sizeof(arr) / sizeof(arr[0])


template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template<typename T>
FORCE_INLINE T sqr(T x)
{
    return x * x;
}

int randRange(int low, int high);

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
