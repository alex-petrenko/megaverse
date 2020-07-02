#include <gtest/gtest.h>

#include <gfx/windowless_context.hpp>


TEST(gfx, context)
{
    // this will only work if the GPU is present
    constexpr int device = 0;
    WindowlessContext context{device};

    EXPECT_EQ(device, context.gpuDevice());
}
