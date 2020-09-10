#include <cstdlib>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <util/tiny_logger.hpp>
#include <util/util.hpp>
#include <util/tiny_profiler.hpp>

#include <env/env.hpp>

#include <v4r_rendering/v4r_env_renderer.hpp>

#include <magnum_rendering/magnum_env_renderer.hpp>


constexpr int delayMs = 10; //1000 / 15;

constexpr bool useVulkan = true;

constexpr bool viz = true;
constexpr bool hires = true;
bool randomActions = true;

constexpr bool performanceTest = !viz;
constexpr int W = hires ? 800 : 128, H = hires ? 450 : 72;
constexpr int maxNumFrames = performanceTest ? 20'000 : 2'000'000'000;
constexpr int maxNumEpisodes = performanceTest ? 2'000'000'000 : 20;

// don't ask me, this is what waitKeyEx returns
constexpr auto keyUp = 65362, keyLeft = 65361, keyRight = 65363, keyDown = 65364;


int main_loop(Env &env, EnvRenderer &renderer)
{
    if constexpr (performanceTest)
        randomActions = true;

    auto activeAgent = 0;
    int numFrames = 0;

    if constexpr (viz) {
        for (int i = 0; i < env.getNumAgents(); ++i) {
            const auto wname = std::to_string(i);
            cv::namedWindow(wname);
            cv::moveWindow(wname, int(W * i * 1.1), 0);
        }
    }

    bool shouldExit = false;
    for (int episode = 0; episode < maxNumEpisodes && !shouldExit; ++episode) {
        bool done;

        tprof().startTimer("reset");
        env.reset();
        renderer.reset(env);
        tprof().pauseTimer("reset");

        while (true) {
            done = env.step();
            renderer.draw(env);

            for (int i = 0; i < env.getNumAgents(); ++i) {
                const uint8_t *obsData = renderer.getObservation(i);

                if constexpr (viz) {
                    cv::Mat mat(H, W, CV_8UC4, (char *) obsData);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

                    if constexpr (!useVulkan)
                        cv::flip(mat, mat, 0);

                    cv::imshow(std::to_string(i), mat);
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

                env.setAction(activeAgent, latestAction);
            }

            if (randomActions) {
                auto randomAction = randRange(0, int(Action::NumActions), env.getRng());
                env.setAction(activeAgent, Action(1 << randomAction));
            }

            ++numFrames;
            if (numFrames > maxNumFrames) {
                shouldExit = true;
                break;
            } else if (numFrames % 5000 == 0)
                TLOG(INFO) << "Progress " << numFrames << "/" << maxNumFrames;

            if (done) {
                TLOG(INFO) << "Finished episode";
                break;
            }
        }
    }

    return numFrames;
}


int main(int argc, char** argv)
{
    (void)argc, void(argv);  // annoying warnings

    const int numAgents = 4;

    Env env(numAgents);
    env.seed(42);
    env.reset();

    std::unique_ptr<EnvRenderer> renderer;
    if constexpr (useVulkan)
        renderer = std::make_unique<V4REnvRenderer>(env, W, H);
    else {
        const auto debugDraw = false;
        renderer = std::make_unique<MagnumEnvRenderer>(env, W, H, debugDraw);
    }

    tprof().startTimer("loop");
    auto nFrames = main_loop(env, *renderer);
    const auto usecPassed = tprof().stopTimer("loop");
    tprof().stopTimer("reset");

    auto fps = nFrames / (usecPassed / 1e6);

    TLOG(DEBUG) << "\n\n" << fps * env.getNumAgents() << " FPS! (" << env.getNumAgents() << "*" << fps << ") for " << nFrames << " frames";

    return EXIT_SUCCESS;
}
