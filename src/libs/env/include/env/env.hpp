#pragma once

#include <random>
#include <vector>

#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <util/util.hpp>

#include <env/agent.hpp>
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
    LookDown = 1 << 7,
    LookUp = 1 << 8,

    Interact = 1 << 9,

    NumActions = 10,
};


inline Action operator|(Action a, Action b) { return Action(int(a) | int(b)); }
inline Action & operator|=(Action &a, Action b) { return (Action &)((int &)a |= int(b)); }
inline Action operator&(Action a, Action b) { return Action(int(a) & int(b)); }
inline Action & operator&=(Action &a, Action b) { return (Action &)((int &)a &= int(b)); }
inline bool operator!(Action a) {return a == Action::Idle; }


typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;


class Env
{
public:
    explicit Env(int seed = -1);

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

    Rng & getRng() { return rng; }

private:

public:
    static constexpr int numAgents = 2;

    Scene3D scene;

    VoxelGrid<VoxelState> grid{100, {0, 0, 0}, 1};
    std::vector<BoundingBox> layoutDrawables;
    BoundingBox exitPad;

    std::vector<VoxelCoords> agentStartingPositions;
    std::vector<std::unique_ptr<Agent>> agents;

private:
    static constexpr auto walkSpeed = 0.66f, strafeSpeed = 0.5f;

    Rng rng{std::random_device{}()};

    std::array<Action, numAgents> currAction;

    LayoutGenerator layoutGenerator{rng};
};