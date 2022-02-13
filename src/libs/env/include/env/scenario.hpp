#pragma once

#include <map>
#include <memory>
#include <string>

#include <util/tiny_logger.hpp>
#include <util/string_utils.hpp>

#include <env/env.hpp>
#include <env/const.hpp>


namespace Megaverse
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
        const auto scenarioNameLowercase = toLower(scenarioName);
        scenarioRegistry[scenarioNameLowercase] = factoryFunc;
        TLOG(INFO) << "Scenario " << scenarioNameLowercase << " registered! " << scenarioRegistry.size() << " scenarios.";
    }

    static std::vector<ScenarioRegistry::key_type> registeredScenarios()
    {
        auto &scenarioRegistry = getScenarioRegistry();
        std::vector<ScenarioRegistry::key_type> keys;
        keys.reserve(scenarioRegistry.size());
        for (const auto& [key, _] : scenarioRegistry)
            keys.emplace_back(key);
        return keys;
    }

    static ScenarioPtr create(const std::string &scenarioName, Env &env, Env::EnvState &envState)
    {
        const auto scenarioNameLowercase = toLower(scenarioName);
        const auto &scenarioRegistry = getScenarioRegistry();

        // TLOG(INFO) << "Num scenarios " << scenarioRegistry.size();
        // TLOG(INFO) << "Registry ptr " << &scenarioRegistry;
        // for (const auto &[k,v]: scenarioRegistry)
        //    TLOG(INFO) << k;

        if (!scenarioRegistry.count(scenarioNameLowercase))
            TLOG(FATAL) << "Unknown scenario " << scenarioNameLowercase << ". Did you register the scenario in scenariosGlobalInit()?";

        const auto factoryFunc = scenarioRegistry.at(scenarioNameLowercase);

        return factoryFunc(scenarioNameLowercase, env, envState);
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
     * Called once after construction.
     * We can't call virtual methods in the ctor, therefore this method can be useful.
     */
    virtual void init()
    {
        initializeDefaultParameters();

        for (int i = 0; i < env.getNumAgents(); ++i)
            rewardShaping[i] = {{Str::teamSpirit, 0.0f}};

        // initialize default reward shaping specific for this scenario
        initRewardShaping();
    }

    /**
     * Called on the episode boundary.
     */
    virtual void reset() {}

    /**
     * Easy way to terminate the episode a bit more smoothly, rather than abruptly (i.e. on the same frame the last
     * reward was collected).
     */
    void doneWithTimer(float remainingTimeSeconds=0.3f)
    {
        envState.currEpisodeSec = std::max(envState.currEpisodeSec, episodeLengthSec() - remainingTimeSeconds);
    }

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
     * Called after every step() by the environment, use to adjust health bars, etc.
     */
    virtual void updateUI() {}

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
     * We display a rudimentary HUD just by drawing a bunch of geometric primitives very close to the camera.
     */
    virtual void addUIDrawables(DrawablesMap &) {}

    /**
     * @return a set of colors used by the renderer in this scenario.
     */
    virtual std::vector<Magnum::Color3> getPalette() const = 0;

    /**
     * @return unshaped reward for the current episode (i.e. 1 for success 0 for failure)
     * Typically called only on the last frame of the episode, when done=True
     */
    virtual float trueObjective(int agentIdx) const = 0;

    /**
     * @return episode duration in seconds, this can be overridden
     */
    virtual float episodeLengthSec() const
    {
        const auto episodeLengthSec = getFloatParams().at(Str::episodeLengthSec);
        return episodeLengthSec;
    }

    /**
     * Each environment should provide the reward shaping dictionary which allows changing rewards through API
     * (even during training)
     */
    virtual RewardShaping defaultRewardShaping() const = 0;

    /**
     * Utility function, used by initRewardShaping() implementations.
     * Replaces rewards in rewardShaping with those provided by rs (or adds them to the map if they don't exist).
     */
    void initRewards(const RewardShaping &rs)
    {
        for (int i = 0; i < env.getNumAgents(); ++i)
            for (const auto &[rewardName, value] : rs)
                rewardShaping[i][rewardName] = value;
    }

    /**
     * Initialize default reward shaping for this scenario. Should be overloaded by all scenarios.
     */
    virtual void initRewardShaping()
    {
        initRewards(defaultRewardShaping());
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
        fp[Str::verticalLookLimitRad] = 0.2f;
        fp[Str::useUIRewardIndicators] = 0.0f;
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

protected:
    /**
     * @return reward for a specific string key for a particular agent in the environment
     * Since different agents can be controlled by different policies, they can also have different reward shaping
     * associated with them (e.g. when we're doing PBT).
     * Therefore we need a separate rewardShaping dictionary for every agent.
     */
    virtual float getReward(const std::string &rewardName, int agentIdx) const
    {
        return rewardShaping[agentIdx].at(rewardName);
    }

    /**
     * Reward agent individually, do not reward other agents even if teamSpirit > 0
     */
    virtual void rewardAgent(const std::string &rewardName, int agentIdx, float multiplier)
    {
        envState.lastReward[agentIdx] += getReward(rewardName, agentIdx) * multiplier;
    }

    /**
     * @return teamSpirit configured for this agent, can be used in collaborative tasks.
     */
    virtual float teamSpirit(int agentIdx) const
    {
        return getReward(Str::teamSpirit, agentIdx);
    }

    /**
     * Which team the agent belongs to. By default all agents are on the same team.
     */
    virtual int teamAffinity(int /*agentIdx*/) const { return 0; }

    virtual int teamSize(const int agentIdx) const {
        const int currTeam = teamAffinity(agentIdx);
        int size = 0;
        for (int i = 0; i < env.getNumAgents(); ++i)
            size += currTeam == teamAffinity(i) ? 1 : 0;

        return size;
    }

    /**
     * Reward all agents, taking teamSpirit into account. Can be useful for collaborative tasks.
     * TeamSpirit is expected to be in [0, 1] range.
     * The agent whose action was rewarded gets the full reward. Other agents get teamSpirit * reward.
     */
    virtual void rewardTeam(const std::string &rewardName, int agentIdx, float multiplier)
    {
        const auto currTeam = teamAffinity(agentIdx);
        rewardAgent(rewardName, agentIdx, multiplier * (1 - teamSpirit(agentIdx)));
        for (int i = 0; i < env.getNumAgents(); ++i)
            if (teamAffinity(i) == currTeam)
                envState.lastReward[i] += getReward(rewardName, i) * teamSpirit(i) * multiplier / teamSize(i);
    }

    /**
     * Reward all agents equally, regardless of team spirit.
     */
    virtual void rewardAll(const std::string &rewardName, float multiplier)
    {
        for (int i = 0; i < env.getNumAgents(); ++i)
            rewardAgent(rewardName, i, multiplier);
    }

private:
    static ScenarioRegistry & getScenarioRegistry()
    {
        static ScenarioRegistry scenarioRegistry;
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
