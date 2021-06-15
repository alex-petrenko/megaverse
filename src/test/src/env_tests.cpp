#include <gtest/gtest.h>

#include <Magnum/GL/Context.h>

#include <env/env.hpp>
#include <scenarios/init.hpp>

#include <magnum_rendering/magnum_env_renderer.hpp>


using namespace Megaverse;


class EnvTest : public ::testing::Test {
protected:
    virtual void SetUp() { scenariosGlobalInit(); }
};


TEST_F(EnvTest, env)
{
    Env env{"TowerBuilding"};
}

TEST_F(EnvTest, multipleEnvs)
{
    Envs envs;
    for (int i = 0; i < 3; ++i) {
        auto e = std::make_unique<Env>("TowerBuilding");
        e->reset();
        envs.emplace_back(std::move(e));
    }

    MagnumEnvRenderer renderer{envs, 128, 72};

    for (int i = 0; i < 3; ++i)
        renderer.reset(*envs[i], i);

    for (int i = 0; i < 3; ++i)
        renderer.draw(envs);
}
