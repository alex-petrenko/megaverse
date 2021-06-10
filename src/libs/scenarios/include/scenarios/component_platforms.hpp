#pragma once

#include <env/scenario_component.hpp>

#include <scenarios/platforms.hpp>



namespace Megaverse
{

class PlatformsComponent : public ScenarioComponent
{
public:
    explicit PlatformsComponent(Scenario &scenario)
        : ScenarioComponent{scenario}
    {
    }

    void reset(Env &, Env::EnvState &) override
    {
        platforms.clear();
        levelRoot.reset();
        levelRoot = std::make_unique<Object3D>(nullptr);
    }

    void addPlatform(std::unique_ptr<Platform> platform)
    {
        platforms.emplace_back(std::move(platform));
    }

public:
    std::vector<std::unique_ptr<Platform>> platforms;
    std::unique_ptr<Object3D> levelRoot;
};

}