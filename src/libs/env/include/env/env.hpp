#pragma once

#include <map>
#include <random>
#include <vector>
#include <algorithm>

#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/util.hpp>

#include <env/agent.hpp>
#include <env/physics.hpp>


namespace VoxelWorld
{

class Scenario;

enum class Action
{
    Idle = 0,

    Left = 1 << 1,
    Right = 1 << 2,

    Forward = 1 << 3,
    Backward = 1 << 4,

    LookLeft = 1 << 5,
    LookRight = 1 << 6,

    Jump = 1 << 7,
    Interact = 1 << 8,

    LookDown = 1 << 9,
    LookUp = 1 << 10,

    NumActions = 11,
};


inline Action operator|(Action a, Action b) { return Action(int(a) | int(b)); }

inline Action &operator|=(Action &a, Action b) { return (Action &) ((int &) a |= int(b)); }

inline Action operator&(Action a, Action b) { return Action(int(a) & int(b)); }

inline Action &operator&=(Action &a, Action b) { return (Action &) ((int &) a &= int(b)); }

inline Action operator~(Action a) { return Action(~int(a)); }

inline bool operator!(Action a) { return a == Action::Idle; }


enum class DrawableType
{
    First = 0,

    Box = 0,
    Capsule = 1,
    Sphere = 2,
    Cone = 3,
    Cylinder = 4,

    NumTypes,
};


struct SceneObjectInfo
{
    SceneObjectInfo(Object3D *objectPtr, const Magnum::Color3 &color)
        : objectPtr{objectPtr}
          , color{color}
    {
    }

    Object3D *objectPtr;
    Magnum::Color3 color;
};


using FloatParams = std::map<std::string, float>;
using Agents = std::vector<AbstractAgent *>;
using DrawablesMap = std::map<DrawableType, std::vector<SceneObjectInfo>>;
using RewardShaping = std::map<std::string, float>;

class Env
{
public:
    /**
     * Physics-related fields (PyBullet)
     */
    struct EnvPhysics
    {
        EnvPhysics()
        {
            // what does this really do?
            bBroadphase.getOverlappingPairCache()->setInternalGhostPairCallback(&ghostPairCallback);
        }

        void reset()
        {
            collisionShapes.clear();
        }

        btGhostPairCallback ghostPairCallback;

        btDbvtBroadphase bBroadphase;
        btSequentialImpulseConstraintSolver bConstraintSolver;
        btDefaultCollisionConfiguration bCollisionConfiguration;
        btCollisionDispatcher bCollisionDispatcher{&bCollisionConfiguration};
        btDiscreteDynamicsWorld bWorld{&bCollisionDispatcher, &bBroadphase, &bConstraintSolver, &bCollisionConfiguration};

        std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;
    };

    /**
     * Current state of the environment.
     * This is what other components (e.g. Scenario) will get access to.
     */
    struct EnvState
    {
    public:
        explicit EnvState(int numAgents)
        : currAction(size_t(numAgents), Action::Idle)
        , lastReward(size_t(numAgents), 0)
        , totalReward(size_t(numAgents), 0.0f)
        {
        }

        void reset()
        {
            done = false;
            currEpisodeSec = 0;
            numFrames = 0;

            std::fill(currAction.begin(), currAction.end(), Action::Idle);
            std::fill(lastReward.begin(), lastReward.end(), 0.0f);
            std::fill(totalReward.begin(), totalReward.end(), 0.0f);

            scene = std::make_unique<Scene3D>();

            agents.clear();

            physics.reset();
        }

    public:
        EnvPhysics physics;

        // Basic environment info
        bool done = false;
        int numFrames = 0;
        float currEpisodeSec = 0;
        float simulationStepSeconds = 1.0f / 15.0f;  // 15 FPS is default
        float lastFrameDurationSec = simulationStepSeconds;
        std::vector<Action> currAction;
        std::vector<float> lastReward, totalReward;

        std::unique_ptr<Scene3D> scene;

        Agents agents;

        Rng rng{std::random_device{}()};
    };

public:
    explicit Env(const std::string &scenarioName, int numAgents = 2, FloatParams customFloatParams = FloatParams{});

    ~Env();

    int getNumAgents() const { return numAgents; }

    Scenario & getScenario() { return *scenario; }

    Scene3D & getScene() { return *state.scene; }

    Agents & getAgents() { return state.agents; }

    EnvPhysics & getPhysics() { return state.physics; }

    /**
     * Main interface between the env and the renderer.
     * @return the list of drawables for each geometric shape supported.
     */
    const DrawablesMap & getDrawables() const { return drawables; }

    void reset();

    /**
     * Set action for the next tick.
     * @param agentIdx index of the agent for which we're setting the action
     * @param action action mask (see enum)
     */
    void setAction(int agentIdx, Action action);

    /**
     * Advance simulation by one step.
     */
    void step();

    bool isDone() const { return state.done; }

    /**
     * @param agentIdx agent for which to query the last reward
     * @return reward in the last tick
     */
    float getLastReward(int agentIdx) const { return state.lastReward[agentIdx]; }

    float getTotalReward(int agentIdx) const { return state.totalReward[agentIdx]; }

    /**
     * Unshaped reward that we're actually trying to maximize.
     */
    float trueObjective() const;

    float episodeLengthSec() const;

    float remainingTimeFraction() const
    {
        const auto len = episodeLengthSec();
        return std::max(0.0f, (len - state.currEpisodeSec) / len);
    }

    /**
     * We need this because of the requirements of the Vulkan renderer (materials have to be known in advance)
     */
    std::vector<Magnum::Color3> getPalette() const;

    /**
     * Seed the rng with specific seed value.
     */
    void seed(int seedValue);

    Rng &getRng() { return state.rng; }

    /**
     * This is when we're running an actual realtime rendering loop with human controls.
     * Should not be used by Gym env interface.
     * @param sec actual duration of the last frame.
     */
    void setFrameDuration(float sec) { state.lastFrameDurationSec = sec; }

    void setSimulationResolution(float sec) { state.simulationStepSeconds = sec; }

public:
    // need better mechanism for this
    static const std::vector<int> actionSpaceSizes;

private:
    std::string scenarioName;
    std::unique_ptr<Scenario> scenario;

    EnvState state;
    int numAgents;
    DrawablesMap drawables;
};


using Envs = std::vector<std::unique_ptr<Env>>;

}