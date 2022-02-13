#include <cstdlib>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <util/util.hpp>
#include <util/argparse.hpp>
#include <util/os_utils.hpp>
#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>

#include <env/env.hpp>
#include <env/vector_env.hpp>

#include <scenarios/init.hpp>

#if !defined(CORRADE_TARGET_APPLE)
#include <v4r_rendering/v4r_env_renderer.hpp>
#endif

#include <magnum_rendering/magnum_env_renderer.hpp>

#include "viewer_args.hpp"

using namespace Megaverse;


// don't ask me, this is what waitKeyEx returns
constexpr auto keyUp = 65362, keyLeft = 65361, keyRight = 65363, keyDown = 65364;


std::string windowName(int envIdx, int agentIdx)
{
    auto wname = std::to_string(envIdx) + std::to_string(agentIdx);
    return wname;
}


int mainLoop(VectorEnv &venv, EnvRenderer &renderer, bool viz, bool performanceTest, bool randomActions,
             int W, int H, bool useVulkan, int delayMs, int maxNumFrames)
{
    if (performanceTest)
        randomActions = true;

    auto activeAgent = 0;
    int numFrames = 0, prevNumFrames = 0;

    const auto numEnvs = int(venv.envs.size()), numAgents = venv.envs.front()->getNumAgents();

    if (viz) {
        for (int envIdx = 0; envIdx < numEnvs; ++envIdx) {
            for (int i = 0; i < numAgents; ++i) {
                const auto wname = windowName(envIdx, i);
                cv::namedWindow(wname);
                cv::moveWindow(wname, int(W * i * 1.1), int(H * envIdx * 1.1));
            }
        }
    }

    venv.reset();

#if defined(ARTIFICIAL_MEMLEAK)
    std::vector<std::vector<int>> vvi;
#endif

    auto rng = venv.envs.front()->getRng();
    tprof().startTimer("fps_period");

    bool shouldExit = false;
    do {
        tprof().startTimer("step");
        venv.step();
        tprof().pauseTimer("step");

        for (int envIdx = 0; envIdx < int(venv.envs.size()); ++envIdx) {
            for (int i = 0; i < venv.envs[envIdx]->getNumAgents(); ++i) {
                const uint8_t *obsData = renderer.getObservation(envIdx, i);

                if (viz) {
                    cv::Mat mat(H, W, CV_8UC4, (char *) obsData);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

                    if (!useVulkan)
                        cv::flip(mat, mat, 0);

                    cv::imshow(windowName(envIdx, i), mat);
                }
            }
        }

        if (viz) {
            auto latestAction = Action::Idle;
            auto key = cv::waitKeyEx(delayMs);

            switch (key) {
                case 'w':
                    latestAction |= Action::Forward;
                    break;
                case 's':
                    latestAction |= Action::Backward;
                    break;
                case 'a':
                    latestAction |= Action::Left;
                    break;
                case 'd':
                    latestAction |= Action::Right;
                    break;
                case ' ':
                    latestAction |= Action::Jump;
                    break;

                case keyLeft:
                    latestAction |= Action::LookLeft;
                    break;
                case keyRight:
                    latestAction |= Action::LookRight;
                    break;
                case keyUp:
                    latestAction |= Action::LookUp;
                    break;
                case keyDown:
                    latestAction |= Action::LookDown;
                    break;

                case '1':
                case '2':
                case '3':
                case '4':
                    activeAgent = key - 1;
                    break;

                default:
                    break;
            }

            venv.envs.front()->setAction(activeAgent, latestAction);
        }

        if (randomActions) {
            for (auto & env : venv.envs) {
                for (int i = 0; i < env->getNumAgents(); ++i) {
                    auto randomAction = randRange(0, int(Action::NumActions), rng);
                    env->setAction(i, Action(1 << randomAction));
                }
            }
        }

        numFrames += numAgents * numEnvs;

        if (numFrames > maxNumFrames) {
            shouldExit = true;
            TLOG(INFO) << "Done: " << numFrames;
            break;
        } else if (numFrames % 5000 == 0) {
            auto elapsedTimeSec = tprof().stopTimer("fps_period") / 1e6;
            tprof().startTimer("fps_period");

            auto approxFps = float(numFrames - prevNumFrames) / elapsedTimeSec;
            prevNumFrames = numFrames;

            double vmUsage, residentSet;
            unixProcessMemUsage(vmUsage, residentSet);

#if defined(ARTIFICIAL_MEMLEAK)
            // test artificial memleak
            vvi.emplace_back(1024 * 1024, 3);
            TLOG(INFO) << std::accumulate(vvi.back().begin(), vvi.back().end(), 0);
#endif
            TLOG(INFO) << "Progress " << numFrames << "/" << maxNumFrames << ". Approx FPS: " << approxFps << ". VM usage: " << (long long)vmUsage << ". RSS: " << (long long)residentSet;
        }
    } while (!shouldExit);

    return numFrames;
}


int main(int argc, char** argv)
{
    scenariosGlobalInit();

    auto parser = viewerStandardArgParse("megaverse_test_app");
    parser.add_description("This app is designed to test the parallel execution engine and batched renderer\n"
                           "by simulating multiple environments at once. This app uses pretty much the same interface\n"
                           "as the Python Gym environment, sans the Python bindings. Whenever there is a problem\n"
                           "with the environment, it is much easier to debug this app directly, rather\n"
                           "than debugging the same code through Python.\n\n"
                           "Example, render 12 agents at the same time:\n"
                           "megaverse_test_app --scenario Collect --visualize --num_envs 4 --num_simulation_threads 1 --num_agents 3 --hires\n\n"
                           "Some performance figures for future reference (on 10-core Intel i9):\n"
                           "megaverse_test_app --scenario Empty --performance_test --num_envs 64 --num_simulation_threads 1 --num_agents 1\n"
                           "yields approximately 75000 FPS\n"
                           "megaverse_test_app --scenario Collect --performance_test --num_envs 64 --num_simulation_threads 1 --num_agents 1\n"
                           "yields approximately 27000 FPS\n");

    parser.add_argument("--num_envs")
        .help("number of parallel environments to simulate")
        .default_value(64)
        .scan<'i', int>();
    parser.add_argument("--num_simulation_threads")
        .help("number of parallel CPU threads to use for Bullet")
        .default_value(1)
        .scan<'i', int>();
    parser.add_argument("--visualize")
        .help("Whether to render multiple environments on screen")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--visualize")
        .help("Whether to render multiple environments on screen")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--delay_ms")
        .help("Delay between rendered frames in milliseconds. Use only with --visualize")
        .default_value(1)
        .scan<'i', int>();
    parser.add_argument("--performance_test")
        .help("Run for a limited number of env frames (currently 200000) to test performance. Uses random actions.")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--hires")
        .help("Render at high resolution. Only use this parameter with --visualize and if the total number of agents is small")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--user_actions")
        .help("Allows the user to control agents (otherwise will use randomly generated actions). Use only with --visualize")
        .default_value(false)
        .implicit_value(true);

    parseArgs(parser, argc, argv);

    const auto scenarioName = parser.get<std::string>("--scenario");
    const auto numAgents = parser.get<int>("--num_agents");
    const bool useVulkanRenderer = !parser.get<bool>("--use_opengl");
    const int numEnvs = parser.get<int>("--num_envs");  // to test vectorized env interface
    const int numSimulationThreads = parser.get<int>("--num_simulation_threads");
    const auto viz = parser.get<bool>("--visualize");
    const auto delayMs = parser.get<int>("--delay_ms");
    const auto performanceTest = parser.get<bool>("--performance_test");
    const auto hires = parser.get<bool>("--hires");
    const bool randomActions = !parser.get<bool>("--user_actions");

    const int W = hires ? 800 : 128, H = hires ? 450 : 72;
    TLOG(INFO) << "Rendering resolution is [" << W << "x" << H << "] per agent";

    const int maxNumFrames = performanceTest ? 400'000 : 2'000'000'000;

    // FloatParams params{{Str::episodeLengthSec, 0.1f}};
    FloatParams params{{}};

    std::vector<std::unique_ptr<Env>> envs;
    for (int i = 0; i < numEnvs; ++i) {
        envs.emplace_back(std::make_unique<Env>(scenarioName, numAgents, params));
        envs[i]->seed(42 + i);
    }

    std::unique_ptr<EnvRenderer> renderer;
    if (useVulkanRenderer)
#if defined (CORRADE_TARGET_APPLE)
        TLOG(ERROR) << "Vulkan not supported on MacOS";
#else
        renderer = std::make_unique<V4REnvRenderer>(envs, W, H, nullptr, false);
#endif
    else {
        constexpr auto debugDraw = false;
        renderer = std::make_unique<MagnumEnvRenderer>(envs, W, H, debugDraw);
    }

    VectorEnv vectorEnv{envs, *renderer, numSimulationThreads};
    vectorEnv.reset();

    tprof().startTimer("loop");
    auto nFrames = mainLoop(vectorEnv, *renderer, viz, performanceTest, randomActions, W, H, useVulkanRenderer, delayMs, maxNumFrames);
    const auto usecPassed = tprof().stopTimer("loop");
    tprof().stopTimer("step");

    vectorEnv.close();

    const auto fps = nFrames / (usecPassed / 1e6);

    TLOG(DEBUG) << "\n\n" << fps << " FPS! " << nFrames << " frames";

    return EXIT_SUCCESS;
}
