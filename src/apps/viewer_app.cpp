#include <Magnum/Timeline.h>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>
#include <scenarios/init.hpp>

#include <viewer/viewer.hpp>

using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace Megaverse;

// TODO: CLI parameters
#if defined(CORRADE_TARGET_APPLE)
constexpr bool useVulkanRenderer = false;

static_assert(!useVulkanRenderer, "Vulkan not supported on MacOS");
#else
constexpr bool useVulkanRenderer = true;
#endif

// "main" envs
//const auto scenarioName = "ObstaclesHard";  // *
//const auto scenarioName = "ObstaclesEasy";  // *
//const auto scenarioName = "Collect";    // *
//const auto scenarioName = "Sokoban";  // *
const auto scenarioName = "TowerBuilding";
//const auto scenarioName = "HexMemory";  // *
//const auto scenarioName = "HexExplore";  // *
//const auto scenarioName = "Rearrange";  // *

// test and experimental envs
//const auto scenarioName = "Test";
//const auto scenarioName = "Empty";
//const auto scenarioName = "BoxAGone";
//const auto scenarioName = "Football";
//const auto scenarioName = "ObstaclesWalls";
//const auto scenarioName = "ObstaclesSteps";
//const auto scenarioName = "ObstaclesLava";


class ViewerApp: public Viewer
{
public:
    explicit ViewerApp(Envs &envs, unsigned int minimalLoopPeriod, const Arguments& arguments);

private:
    void drawEvent() override;

    void tickEvent() override;

    void keyPressEvent(KeyEvent &event) override;
    void keyReleaseEvent(KeyEvent &event) override;

    void handleActions(const KeyEvent::Key &key, bool addAction);

private:
    Action currAction = Action::Idle;

    Timeline timeline;
};

ViewerApp::ViewerApp(Envs &envs, unsigned int minimalLoopPeriod, const Arguments& arguments)
: Viewer{envs, useVulkanRenderer, nullptr, arguments}
{
    setMinimalLoopPeriod(minimalLoopPeriod);

    timeline.start();
}

void ViewerApp::tickEvent()
{
    auto &env = envs[activeEnv];
    env->setFrameDuration(timeline.previousFrameDuration());
    env->setAction(activeAgent, currAction);

    currAction &= ~Action::Interact;

    env->step();
    const auto done = env->isDone();

    if (done) {
        TLOG(INFO) << "Done!";

        std::ostringstream s;
        for (int i = 0; i < env->getNumAgents(); ++i) s << " " << env->getTotalReward(i);
        TLOG(INFO) << "Total reward: " << s.str();
        s.clear();

        for (int i = 0; i < env->getNumAgents(); ++i) s << " " << env->trueObjective(i);
        TLOG(INFO) << "True objectives: " << s.str();

        env->reset();
    }

    Viewer::step(std::vector<bool>(1, done));
}

void ViewerApp::drawEvent()
{
    Viewer::drawEvent();
    timeline.nextFrame();
}

void ViewerApp::keyPressEvent(KeyEvent &event)
{
    Viewer::keyPressEvent(event);

    handleActions(event.key(), true);
    event.setAccepted();
}

void ViewerApp::keyReleaseEvent(KeyEvent &event)
{
    Viewer::keyReleaseEvent(event);

    handleActions(event.key(), false);
    event.setAccepted();
}

void ViewerApp::handleActions(const KeyEvent::Key &key, bool addAction)
{
    auto a = Action::Idle;

    switch(key) {
        case KeyEvent::Key::W: a = Action::Forward; break;
        case KeyEvent::Key::S: a = Action::Backward; break;
        case KeyEvent::Key::A: a = Action::Left; break;
        case KeyEvent::Key::D: a = Action::Right; break;

        case KeyEvent::Key::Left: a = Action::LookLeft; break;
        case KeyEvent::Key::Right: a = Action::LookRight; break;
        case KeyEvent::Key::Up: a = Action::LookUp; break;
        case KeyEvent::Key::Down: a = Action::LookDown; break;

        case KeyEvent::Key::Space: a = Action::Jump; break;

        case KeyEvent::Key::E: a = Action::Interact; break;

        default: break;
    }

    if (a == Action::Idle)
        return;

    if (addAction)
        currAction |= a;
    else
        currAction &= ~a;
}


int main(int argc, char** argv)
{
    scenariosGlobalInit();

    const int numAgents = 1;

    FloatParams params{{Str::useUIRewardIndicators, 1.0f}};
    auto env = std::make_unique<Env>(scenarioName, numAgents, params);
#ifdef FIXED_SEED
    env->seed(42);
#endif
    env->reset();

    const int desiredFps = 15;
    const unsigned int delayMs = 1000 / desiredFps;

    env->setSimulationResolution(1.0f / desiredFps);

    Envs envs;
    envs.emplace_back(std::move(env));

    ViewerApp app(envs, delayMs, {argc, argv});
    return app.exec();
}