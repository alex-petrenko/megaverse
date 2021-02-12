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
    struct UIElement
    {
    public:
        UIElement() = default;

        UIElement(Object3D *anchor, Object3D *uiElement)
        : anchor{anchor}
        , uiElement{uiElement}
        {
        }

        void rescale(const Magnum::Vector3 &requiredScale) const
        {
            const auto scale = uiElement->transformation().scaling();
            uiElement->scaleLocal(requiredScale / scale);
        }

        void show()
        {
            if (visible)
                return;

            anchor->translate({-1000, 0, 0});
            visible = true;
        }

        void hide()
        {
            if (!visible)
                return;

            anchor->translate({1000, 0, 0});
            visible = false;
        }

    public:
        bool visible = true;
        Object3D *anchor = nullptr;
        Object3D *uiElement = nullptr;
    };

    struct DefaultUI
    {
        std::vector<UIElement> remainingTimeBars, positiveRewardIndicator, negativeRewardIndicator;
        float initialRemainingTimeBarScale = 0.24f;
    };

public:
    explicit DefaultScenario(const std::string &scenarioName, Env &env, Env::EnvState &envState)
    : Scenario{scenarioName, env, envState}
    {
        const auto numAgents = env.getNumAgents();
        defaultUI.remainingTimeBars.resize(numAgents);
        defaultUI.positiveRewardIndicator.resize(numAgents), defaultUI.negativeRewardIndicator.resize(numAgents);
    }

    /**
     * Override this if your scenario needs agents of a different type or a different logic for spawning agents.
     * @param agents resulting vector of agents
     */
    void spawnAgents(std::vector<AbstractAgent *> &agents) override
    {
        const auto numAgents = env.getNumAgents();
        const auto verticalLookLimitRad = floatParams[Str::verticalLookLimitRad];
        const auto agentPositions = agentStartingPositions();

        for (int i = 0; i < numAgents; ++i) {
            auto randomRotation = frand(envState.rng) * Magnum::Constants::pi() * 2;
            auto &agent = envState.scene->addChild<DefaultKinematicAgent>(
                envState.scene.get(), envState.physics->bWorld,
                Magnum::Vector3{agentPositions[i]} + Magnum::Vector3{0.5, 0.0, 0.5},
                randomRotation, verticalLookLimitRad
            );
            agent.updateTransform();

            agents.emplace_back(&agent);
        }
    }

    void addEpisodeAgentsDrawables(DrawablesMap &drawables) override
    {
        const auto numAgents = env.getNumAgents();

        // drawing agent's body in single-agent envs is still useful because of the overview camera
#ifdef DONT_DRAW_SELF
        if (numAgents <= 1) {
            // agent can't see itself, so we can avoid wasting resources on rendering it
            return;
        }
#endif

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

    void addUIDrawables(DrawablesMap &drawables) override
    {
        const auto numAgents = env.getNumAgents();
        for (int i = 0; i < numAgents; ++i) {
            auto agent = envState.agents[i];

            auto &uiObject = agent->getCameraObject()->addChild<Object3D>();
            uiObject.translate({0, 0, -0.2f});

            auto &remainingTimeBarAnchor = uiObject.addChild<Object3D>();
            remainingTimeBarAnchor.translate({0, -0.131f, 0});
            auto &remainingTimeBar = remainingTimeBarAnchor.addChild<Object3D>();
            remainingTimeBar.scaleLocal({defaultUI.initialRemainingTimeBarScale, 0.0015, 0.001});
            drawables[DrawableType::Box].emplace_back(&remainingTimeBar, rgb(ColorRgb::BLUE));
            defaultUI.remainingTimeBars[i] = UIElement{&remainingTimeBarAnchor, &remainingTimeBar};

            if (floatParams[Str::useUIRewardIndicators] > 0) {
                auto addRewardIndicator = [&](float xOffset, ColorRgb color, std::vector<UIElement> &container) {
                    auto &indicatorAnchor = uiObject.addChild<Object3D>();
                    indicatorAnchor.translate({xOffset, 0, 0});
                    auto &indicator = indicatorAnchor.addChild<Object3D>();
                    indicator.scaleLocal({0.01, 0.03, 0.0001});
                    drawables[DrawableType::Box].emplace_back(&indicator, rgb(color));
                    container[i] = UIElement{&indicatorAnchor, &indicator};
                    container[i].hide();
                };

                addRewardIndicator(-0.23f, ColorRgb::GREEN, defaultUI.positiveRewardIndicator);
                addRewardIndicator(0.23f, ColorRgb::RED, defaultUI.negativeRewardIndicator);
            }
        }
    }

    void updateUI() override
    {
        const auto numAgents = env.getNumAgents();
        for (int i = 0; i < numAgents; ++i) {
            auto &bar = defaultUI.remainingTimeBars[i];
            bar.rescale({env.remainingTimeFraction() * defaultUI.initialRemainingTimeBarScale, 0.0015, 0.001});

            if (floatParams[Str::useUIRewardIndicators] > 0) {
                if (envState.lastReward[i] > FLT_EPSILON) {
                    defaultUI.positiveRewardIndicator[i].show();
                    defaultUI.positiveRewardIndicator[i].rescale({0.06, 0.04f * envState.lastReward[i], 0.0001});
                    defaultUI.negativeRewardIndicator[i].hide();
                } else if (envState.lastReward[i] < -FLT_EPSILON) {
                    defaultUI.negativeRewardIndicator[i].show();
                    defaultUI.negativeRewardIndicator[i].rescale({0.06, -0.04f * envState.lastReward[i], 0.0001});
                    defaultUI.positiveRewardIndicator[i].hide();
                } else {
                    defaultUI.positiveRewardIndicator[i].hide();
                    defaultUI.negativeRewardIndicator[i].hide();
                }
            }
        }
    }

    std::vector<Magnum::Color3> getPalette() const override
    {
        std::vector<Magnum::Color3> palette;
        for (auto c : allColors)
            palette.emplace_back(rgb(c));

        return palette;
    }

protected:
    DefaultUI defaultUI;
};

}
