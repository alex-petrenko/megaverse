#include <cstring>

#include <util/util.hpp>


int randRange(int low, int high)
{
    return rand() % (high - low) + low;
}

void memcpyStride(char *dst, const char *src, int elemSize, int numElements, int ofs, int stride)
{
    for (int i = 0, dstOfs = 0, srcOfs = ofs; i < numElements; ++i, dstOfs += elemSize, srcOfs += stride)
        memcpy(dst + dstOfs, src + srcOfs, elemSize);
}
