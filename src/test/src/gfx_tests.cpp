#include <gtest/gtest.h>

#include <magnum_rendering/windowless_context.hpp>


TEST(gfx, context)
{
    // this will only work if the GPU is present
    constexpr int device = 0;
    WindowlessContext context{device};
}
