#include <gtest/gtest.h>

#include <magnum_rendering/rendering_context.hpp>

using namespace Megaverse;


TEST(gfx, context)
{
    // this will only work if the GPU is present
    constexpr int device = 0;
    WindowlessContext context{device};
}
