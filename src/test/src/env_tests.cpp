#include <gtest/gtest.h>

#include <env/env.hpp>
#include <magnum_rendering/magnum_env_renderer.hpp>
#include <Magnum/GL/Context.h>


TEST(env, layout)
{
    Rng rng{std::random_device{}()};

    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    LayoutGenerator lg{rng};

    lg.init(1, LayoutType::Empty);
    lg.generate(grid);

    const auto ptr = grid.get({0, 0, 0});
    EXPECT_NE(ptr, nullptr);
    EXPECT_TRUE(ptr->solid);

    auto parallelepipeds = lg.extractPrimitives(grid);
    EXPECT_FALSE(parallelepipeds.empty());
    EXPECT_EQ(parallelepipeds.size(), 5);
}

TEST(env, env)
{
    Env env;
}

TEST(env, multipleEnvs)
{
    Env env1;
    Env env2;

    env1.reset();
    MagnumEnvRenderer renderer1{env1, 128, 72};
    renderer1.reset(env1);
    renderer1.draw(env1);

    env2.reset();
    MagnumEnvRenderer renderer2{env2, 128, 72};
    renderer2.reset(env2);
    renderer2.draw(env2);
}
