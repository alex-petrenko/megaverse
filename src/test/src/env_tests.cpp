#include <gtest/gtest.h>

#include <env/env.hpp>


TEST(env, layout)
{
    Rng rng{std::random_device{}()};

    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    LayoutGenerator lg{rng};

    lg.init();
    lg.generateFloorWalls(grid);

    const auto ptr = grid.get({0, 0, 0});
    EXPECT_NE(ptr, nullptr);
    EXPECT_TRUE(ptr->solid);

    auto parallelepipeds = lg.extractPrimitives(grid);
    EXPECT_FALSE(parallelepipeds.empty());
}

TEST(env, env)
{
    Env env;
}
