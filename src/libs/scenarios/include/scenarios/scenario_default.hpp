#pragma once

#include <util/magnum.hpp>

#include <env/scenario.hpp>


namespace VoxelWorld
{

/**
 * Default base class for scenarios containing some boilerplate code.
 * Allows us to avoid code duplication for the most standard things.
 * Scenarios don't have to inherit from DefaultScenario, they are free to inherit directly from scenario to retain
 * full flexibility.
 */
class DefaultScenario : public Scenario
{
public:
    explicit DefaultScenario(const std::string &scenarioName, Env &env, Env::EnvState &envState)
    : Scenario{scenarioName, env, envState}
    {
    }

    void spawnAgents(std::vector<AbstractAgent *> &agents) override
    {
        const auto numAgents = env.getNumAgents();
        const auto verticalLookLimitRad = floatParams[Str::verticalLookLimitRad];
        const auto agentPositions = agentStartingPositions();

        for (int i = 0; i < numAgents; ++i) {
            auto randomRotation = frand(envState.rng) * Magnum::Constants::pi() * 2;
            auto &agent = envState.scene->addChild<DefaultKinematicAgent>(
                envState.scene.get(), envState.physics.bWorld,
                Magnum::Vector3{agentPositions[i]} + Magnum::Vector3{0.5, 0.0, 0.5},
                randomRotation, verticalLookLimitRad
            );

            agents.emplace_back(&agent);
        }
    }

    void addEpisodeAgentsDrawables(DrawablesMap &drawables) override
    {
        const auto numAgents = env.getNumAgents();
        if (numAgents <= 1) {
            // agent can't see itself, so we can avoid wasting resources on rendering it
            return;
        }

        for (int i = 0; i < numAgents; ++i) {
            auto agent = envState.agents[i];

            auto &agentBody = agent->addChild<Object3D>();
            agentBody.scale({0.35f, 0.36f, 0.35f}).translate({0, 0.09f, 0});

            drawables[DrawableType::Capsule].emplace_back(&agentBody, rgb(agentColors[i % numAgentColors]));

            auto &eyesObject = agent->getCameraObject()->addChild<Object3D>();
            eyesObject.scale({0.25f, 0.12f, 0.2f}).translate({0.0f, 0.0f, -0.19f});

            drawables[DrawableType::Box].emplace_back(&eyesObject, rgb(ColorRgb::AGENT_EYES));

            // visualize location where agent interacts with objects
            // auto &pickupSpot = agent.pickupSpot->addChild<Object3D>();
            // pickupSpot.scale({0.03f, 0.03f, 0.03f});
            // addStandardDrawable(DrawableType::Box, pickupSpot, 0x222222_rgbf);
        }
    }

    std::vector<Magnum::Color3> getPalette() const override
    {
        std::vector<Magnum::Color3> palette;
        for (auto c : allColors)
            palette.emplace_back(rgb(c));

        return palette;
    }
};

}
