#include <cstdlib>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <util/tiny_logger.hpp>
#include <util/util.hpp>
#include <util/tiny_profiler.hpp>

#include <env/env.hpp>
#include <env/const.hpp>
#include <env/vector_env.hpp>

#include <scenarios/init.hpp>

#include <v4r_rendering/v4r_env_renderer.hpp>

#include <magnum_rendering/magnum_env_renderer.hpp>


using namespace VoxelWorld;


constexpr int delayMs = 20;  // 1000 / 15;

constexpr ConstStr scenario = "Collect";

constexpr bool useVulkan = true;

constexpr bool viz = true;
constexpr bool hires = true;
bool randomActions = true;

constexpr bool performanceTest = !viz;
constexpr int W = hires ? 800 : 128, H = hires ? 450 : 72;
constexpr int maxNumFrames = performanceTest ? 8'000 : 2'000'000'000;
constexpr int maxNumEpisodes = performanceTest ? 2'000'000'000 : 20;

// don't ask me, this is what waitKeyEx returns
constexpr auto keyUp = 65362, keyLeft = 65361, keyRight = 65363, keyDown = 65364;


std::string windowName(int envIdx, int agentIdx)
{
    const auto wname = std::to_string(envIdx) + std::to_string(agentIdx);
    return wname;
}


int mainLoop(VectorEnv &venv, EnvRenderer &renderer)
{
    if constexpr (performanceTest)
        randomActions = true;

    auto activeAgent = 0;
    int numFrames = 0;

    if constexpr (viz) {
        for (int envIdx = 0; envIdx < int(venv.envs.size()); ++envIdx) {
            for (int i = 0; i < venv.envs[envIdx]->getNumAgents(); ++i) {
                const auto wname = windowName(envIdx, i);
                cv::namedWindow(wname);
                cv::moveWindow(wname, int(W * i * 1.1), int(H * envIdx * 1.1));
            }
        }
    }

    venv.reset();

    bool shouldExit = false;

    while (!shouldExit) {
        tprof().startTimer("step");
        venv.step();
        tprof().pauseTimer("step");

        for (int envIdx = 0; envIdx < int(venv.envs.size()); ++envIdx) {
            if (venv.done[envIdx])
                TLOG(INFO) << "Episode boundary env: " << envIdx << " frames: " << numFrames;

            for (int i = 0; i < venv.envs[envIdx]->getNumAgents(); ++i) {
                const uint8_t *obsData = renderer.getObservation(envIdx, i);

                if constexpr (viz) {
                    cv::Mat mat(H, W, CV_8UC4, (char *) obsData);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

                    if constexpr (!useVulkan)
                        cv::flip(mat, mat, 0);

                    cv::imshow(windowName(envIdx, i), mat);
                }
            }
        }

        if constexpr (viz) {
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
                    activeAgent = 0;
                    break;
                case '2':
                    activeAgent = 1;
                    break;

                default:
                    break;
            }

            venv.envs.front()->setAction(activeAgent, latestAction);
        }

        if (randomActions) {
            auto randomAction = randRange(0, int(Action::NumActions), venv.envs.front()->getRng());
            venv.envs.front()->setAction(activeAgent, Action(1 << randomAction));
        }

        ++numFrames;
        if (numFrames > maxNumFrames) {
            shouldExit = true;
            TLOG(INFO) << "Done: " << numFrames;
            break;
        } else if (numFrames % 5000 == 0)
            TLOG(INFO) << "Progress " << numFrames << "/" << maxNumFrames;
    }

    return numFrames;
}


int main(int argc, char** argv)
{
    (void)argc, void(argv);  // annoying warnings

    scenariosGlobalInit();

    const int numEnvs = 1;  // to test vectorized env interface
    const int numAgentsPerEnv = 1;
    const int numSimulationThreads = 1;

    FloatParams params{{Str::episodeLengthSec, 100.0f}};

    std::vector<std::unique_ptr<Env>> envs;
    for (int i = 0; i < numEnvs; ++i) {
        envs.emplace_back(std::make_unique<Env>(scenario, numAgentsPerEnv, params));
        envs[i]->seed(42 + i);
    }

    std::unique_ptr<EnvRenderer> renderer;
    if constexpr (useVulkan)
        renderer = std::make_unique<V4REnvRenderer>(envs, W, H);
    else {
        const auto debugDraw = false;
        renderer = std::make_unique<MagnumEnvRenderer>(envs, W, H, debugDraw);
    }

    VectorEnv vectorEnv{envs, *renderer, numSimulationThreads};
    vectorEnv.reset();

    tprof().startTimer("loop");
    auto nFrames = mainLoop(vectorEnv, *renderer);
    const auto usecPassed = tprof().stopTimer("loop");
    tprof().stopTimer("step");

    vectorEnv.close();

    const auto fps = nFrames / (usecPassed / 1e6);
    const auto totalNumAgents = envs.front()->getNumAgents() * envs.size();

    TLOG(DEBUG) << "\n\n" << fps * totalNumAgents << " FPS! (" << totalNumAgents << "*" << fps << ") for " << nFrames << " frames";

    return EXIT_SUCCESS;
}
