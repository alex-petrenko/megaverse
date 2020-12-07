#include <scenarios/scenario_box_a_gone.hpp>


using namespace VoxelWorld;


class BoxAGoneScenario::BoxAGonePlatform : public EmptyPlatform
{
public:
    explicit BoxAGonePlatform(Object3D *parent, Rng &rng, int walls, int numAgents)
    : EmptyPlatform(parent, rng, walls)
    , numAgents{numAgents}
    {
    }

    ~BoxAGonePlatform() = default;

    void init() override
    {
        height = 10;
        length = width = 30;


        for (int i = 0; i < numAgents; ++i)
            agentSpawnCoords.emplace_back(3, 3, 3);
    }

    void generate() override
    {
        EmptyPlatform::generate();
    }

    std::vector<Magnum::Vector3> agentSpawnPoints(int /*numAgents*/) override
    {
        return agentSpawnCoords;
    }

private:
    std::vector<Magnum::Vector3> agentSpawnCoords;

    int numAgents{};
};


BoxAGoneScenario::BoxAGoneScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this}
, platformsComponent{*this}
, fallDetection{*this, vg.grid, *this}
{
}

BoxAGoneScenario::~BoxAGoneScenario() = default;

void BoxAGoneScenario::reset()
{
    vg.reset(env, envState);
    platformsComponent.reset(env, envState);
    fallDetection.reset(env, envState);

    platform = std::make_unique<BoxAGonePlatform>(platformsComponent.levelRoot.get(), envState.rng, WALLS_ALL, env.getNumAgents());
    platform->init(), platform->generate();
    vg.addPlatform(*platform, false);
}

void BoxAGoneScenario::step()
{
}

std::vector<Magnum::Vector3> BoxAGoneScenario::agentStartingPositions()
{
    return platform->agentSpawnPoints(env.getNumAgents());
}

void BoxAGoneScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();
    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType);
}
