#pragma once

#include <map>
#include <memory>
#include <string>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>
#include <env/const.hpp>


namespace VoxelWorld
{

class ScenarioComponent;

class Scenario
{
    friend class ScenarioComponent;

public:
    using ScenarioPtr = std::unique_ptr<Scenario>;
    using FactoryFunc = ScenarioPtr (*)(const std::string &, Env &, Env::EnvState &);
    using ScenarioRegistry = std::map<std::string, FactoryFunc>;

public:
    explicit Scenario(std::string scenarioName, Env &env, Env::EnvState &envState)
    : scenarioName{std::move(scenarioName)}
    , env{env}
    , envState{envState}
    , rewardShaping(size_t(env.getNumAgents()))
    {
    }

    virtual ~Scenario() = default;

/**
 * Scenario registry and factory methods for instantiating custom scenarios.
 */
public:
    static void registerScenario(const std::string &scenarioName, FactoryFunc factoryFunc)
    {
        auto &scenarioRegistry = getScenarioRegistry();
        scenarioRegistry[scenarioName] = factoryFunc;
        TLOG(INFO) << "Scenario " << scenarioName << " registered!";
        TLOG(INFO) << "Num scenarios " << scenarioRegistry.size();
        TLOG(INFO) << "Registry ptr " << &scenarioRegistry;
    }

    static ScenarioPtr create(const std::string &scenarioName, Env &env, Env::EnvState &envState)
    {
        const auto &scenarioRegistry = getScenarioRegistry();

        TLOG(INFO) << "Num scenarios " << scenarioRegistry.size();
        TLOG(INFO) << "Registry ptr " << &scenarioRegistry;
        for (const auto &[k,v]: scenarioRegistry)
            TLOG(INFO) << k;

        if (!scenarioRegistry.count(scenarioName))
            TLOG(FATAL) << "Unknown scenario " << scenarioName << ". Did you register the scenario in scenariosGlobalInit()?";

        const auto factoryFunc = scenarioRegistry.at(scenarioName);

        return factoryFunc(scenarioName, env, envState);
    }

    template<typename ScenarioType>
    static std::unique_ptr<Scenario> scenarioFactory(const std::string &scenarioName, Env &env, Env::EnvState &envState)
    {
        ScenarioPtr p = std::make_unique<ScenarioType>(scenarioName, env, envState);
        return p;
    }

/**
 * Interface between environment and scenario objects.
 */
public:
    /**
     * Called on the episode boundary.
     */
    virtual void reset() {}

    /**
     * This is called by the environment before the physics simulation step.
     */
    virtual void preStep() {}

    /**
     * This is called by the environment after the physics simulation step.
     * This method should normally contain main logic of the scenario.
     */
    virtual void step() {}

    /**
     * @return vector with starting positions of the agents.
     */
    virtual std::vector<Magnum::Vector3> agentStartingPositions() = 0;

    /**
     * Called by the env to generate the agent objects for the episode.
     */
    virtual void spawnAgents(std::vector<AbstractAgent *> &) = 0;

    /**
     * Called by the env to query the set of drawables for the episode.
     */
    virtual void addEpisodeDrawables(DrawablesMap &) {}

    /**
     * Same as above, but for drawables related to agents
     */
    virtual void addEpisodeAgentsDrawables(DrawablesMap &) {}

    /**
     * @return a set of colors used by the renderer in this scenario.
     */
    virtual std::vector<Magnum::Color3> getPalette() const = 0;

    /**
     * @return unshaped reward for the current episode (i.e. 1 for success 0 for failure)
     * Typically called only on the last frame of the episode, when done=True
     */
    virtual float trueObjective() const = 0;

    /**
     * @return episode duration in seconds, this can be overridden
     */
    virtual float episodeLengthSec() const
    {
        const auto episodeLengthSec = getFloatParams().at(Str::episodeLengthSec);
        return episodeLengthSec;
    }

    /**
     * @param agentIdx
     * @return current reward shaping for the agent
     */
    virtual RewardShaping & getRewardShaping(int agentIdx) { return rewardShaping[agentIdx]; }

    /**
     * @param agentIdx
     * @param rs new reward shaping for the agent
     */
    virtual void setRewardShaping(int agentIdx, const RewardShaping &rs) { rewardShaping[agentIdx] = rs; }

/**
 * Other utility functions.
 */
public:
    /**
     * Polymorphically initialize params, if derived scenarios have custom default parameters they should override
     * this method.
     */
    virtual void initializeDefaultParameters()
    {
        auto &fp = floatParams;
        fp[Str::episodeLengthSec] = 60.0f;
        fp[Str::verticalLookLimitRad] = 0.0f;
    }

    virtual const FloatParams & getFloatParams() const { return floatParams; }

    /**
     * This default behavior should typically be sufficient, although can also be overridden.
     */
    virtual void setCustomParameters(const FloatParams &customFloatParams)
    {
        for (const auto &[k, v] : customFloatParams)
            floatParams[k] = v;
    }

private:

    static ScenarioRegistry & getScenarioRegistry()
    {
        static ScenarioRegistry scenarioRegistry;
        static int counter = 0;
        ++counter;
        TLOG(INFO) << counter << " " << &scenarioRegistry;
        return scenarioRegistry;
    }

protected:
    std::string scenarioName;

    // default values, can be overridden in ctor
    FloatParams floatParams;

    /**
     * Adds an optional mechanism for components to access each other.
     * Scenarios may decide to register or not to register their components here.
     */
    std::vector<ScenarioComponent *> components;

    Env &env;
    Env::EnvState &envState;

    // reward shaping schemes for every agent in the env
    std::vector<RewardShaping> rewardShaping;


};

}