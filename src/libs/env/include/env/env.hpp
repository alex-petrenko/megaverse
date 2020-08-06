#pragma once

#include <random>
#include <vector>

#include <Magnum/Timeline.h>  // TODO: remove
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/util.hpp>

#include <env/agent.hpp>
#include <env/physics.hpp>
#include <env/layout_generator.hpp>


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
inline Action & operator|=(Action &a, Action b) { return (Action &)((int &)a |= int(b)); }
inline Action operator&(Action a, Action b) { return Action(int(a) & int(b)); }
inline Action & operator&=(Action &a, Action b) { return (Action &)((int &)a &= int(b)); }
inline Action operator~(Action a) { return Action(~int(a)); }
inline bool operator!(Action a) {return a == Action::Idle; }


typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;


class Env
{
public:
    explicit Env(int numAgents = 2);

    int getNumAgents() const { return numAgents; }

    void reset();

    /**
     * Set action for the next tick.
     * @param agentIdx index of the agent for which we're setting the action
     * @param action action mask (see enum)
     */
    void setAction(int agentIdx, Action action);

    /**
     * Advance simulation by one step.
     * @return episode termination flag
     */
    bool step();

    /**
     * @param agentIdx agent for which to query the last reward
     * @return reward in the last tick
     */
    float getLastReward(int agentIdx)
    {
        return lastReward[agentIdx];
    }

    /**
     * Seed the rng with specific seed value.
     */
    void seed(int seedValue);

    Rng & getRng() { return rng; }

    /**
     * This is when we're running an actual realtime rendereing loop with human controls.
     * Should not be used by Gym env interface.
     * @param sec actual duration of the last frame.
     */
    void setFrameDuration(float sec) { lastFrameDurationSec = sec; }

    void setSimulationResolution(float sec) { simulationStepSeconds = sec; }

public:
    // physics stuff
    btDbvtBroadphase bBroadphase;
    btSequentialImpulseConstraintSolver bConstraintSolver;
    btDefaultCollisionConfiguration bCollisionConfiguration;
    btCollisionDispatcher bCollisionDispatcher{&bCollisionConfiguration};
    btDiscreteDynamicsWorld bWorld{&bCollisionDispatcher, &bBroadphase, &bConstraintSolver, &bCollisionConfiguration};

    std::unique_ptr<Scene3D> scene;

    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    std::vector<BoundingBox> layoutDrawables;
    BoundingBox exitPad;

    std::vector<VoxelCoords> agentStartingPositions;
    std::vector<Agent *> agents;

    std::vector<Object3D *> layoutObjects;

private:
    int numAgents;

    const float horizonSec = 30;
    float episodeDurationSec = 0;

    Rng rng{std::random_device{}()};

    std::vector<Action> currAction;
    std::vector<float> lastReward;

    LayoutGenerator layoutGenerator{rng};

    std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;

    float simulationStepSeconds = 1.0f / 15.0f;
    float lastFrameDurationSec = simulationStepSeconds;
};
