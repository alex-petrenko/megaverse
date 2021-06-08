#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <opencv2/core/mat.hpp>

#include <util/tiny_logger.hpp>

#include <env/env.hpp>

#include <scenarios/init.hpp>

#include <magnum_rendering/magnum_env_renderer.hpp>

#ifndef CORRADE_TARGET_APPLE
    #include <v4r_rendering/v4r_env_renderer.hpp>
#endif

#ifdef WITH_GUI
    #include <viewer/viewer.hpp>
#endif


namespace py = pybind11;

using namespace VoxelWorld;


void setVoxelEnvLogLevel(int level)
{
    setLogLevel(LogLevel(level));
}


class VoxelEnvGym
{
public:
    VoxelEnvGym(
        const std::string &scenario,
        int w, int h,
        int numEnvs, int numAgentsPerEnv, int numSimulationThreads,
        bool useVulkan,
        const std::map<std::string, float> &floatParams
    )
        : numEnvs{numEnvs}
          , numAgentsPerEnv{numAgentsPerEnv}
          , useVulkan{useVulkan}
          , w{w}
          , h{h}
          , numSimulationThreads{numSimulationThreads}
    {
        scenariosGlobalInit();

        for (int i = 0; i < numEnvs; ++i)
            envs.emplace_back(std::make_unique<Env>(scenario, numAgentsPerEnv, floatParams));

        rewards = std::vector<float>(size_t(numEnvs * numAgentsPerEnv));
    }

    void seed(int seedValue)
    {
        TLOG(INFO) << "Seeding vector env with seed value " << seedValue;

        rng.seed((unsigned long) seedValue);
        for (auto &e : envs) {
            const auto noise = randRange(0, 1 << 30, rng);
            e->seed(noise);
        }
    }

    int numAgents() const
    {
        return envs.front()->getNumAgents();
    }

    void reset()
    {
        if (!vectorEnv) {
            if (useVulkan)
#ifdef CORRADE_TARGET_APPLE
                TLOG(ERROR) << "Vulkan not supported on MacOS";
#else
                renderer = std::make_unique<V4REnvRenderer>(envs, w, h, nullptr, false);
#endif
            else
                renderer = std::make_unique<MagnumEnvRenderer>(envs, w, h);

            vectorEnv = std::make_unique<VectorEnv>(envs, *renderer, numSimulationThreads);
        }

        // this also resets the main renderer
        vectorEnv->reset();
    }

    std::vector<int> actionSpaceSizes() const
    {
        return Env::actionSpaceSizes;
    }

    void setActions(int envIdx, int agentIdx, std::vector<int> actions)
    {
        int actionIdx = 0, actionMask = 0;
        const auto &spaces = Env::actionSpaceSizes;

        for (int i = 0; i < int(actions.size()); ++i) {
            const auto action = actions[i];

            if (action > 0)
                actionMask = actionMask | (1 << (actionIdx + action));

            const auto numNonIdleActions = spaces[i] - 1;
            actionIdx += numNonIdleActions;
        }

        envs[envIdx]->setAction(agentIdx, Action(actionMask));
    }

    void step()
    {
        vectorEnv->step();
    }

    bool isDone(int envIdx)
    {
        return vectorEnv->done[envIdx];
    }

    std::vector<float> getLastRewards()
    {
        int i = 0;

        for (int envIdx = 0; envIdx < numEnvs; ++envIdx)
            for (int agentIdx = 0; agentIdx < numAgentsPerEnv; ++agentIdx)
                rewards[i++] = envs[envIdx]->getLastReward(agentIdx);

        return rewards;
    }

    py::array_t<uint8_t> getObservation(int envIdx, int agentIdx)
    {
        const uint8_t *obsData = renderer->getObservation(envIdx, agentIdx);
        return py::array_t<uint8_t>({h, w, 4}, obsData, py::none{});  // numpy object does not own memory
    }

    /**
     * Call this before the first call to render()
     */
    void setRenderResolution(int hiresW, int hiresH)
    {
        renderW = hiresW;
        renderH = hiresH;
    }

    void drawHires()
    {
        if (!hiresRenderer) {
            if (useVulkan)
#ifdef CORRADE_TARGET_APPLE
                TLOG(ERROR) << "Vulkan not supported on MacOS";
#else
                hiresRenderer = std::make_unique<V4REnvRenderer>(envs, renderW, renderH, dynamic_cast<V4REnvRenderer *>(renderer.get()), true);
#endif
            else
                hiresRenderer = std::make_unique<MagnumEnvRenderer>(envs, renderW, renderH);

            for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx)
                hiresRenderer->reset(*envs[envIdx], envIdx);
        }

        for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
            if (isDone(envIdx))
                hiresRenderer->reset(*envs[envIdx], envIdx);

            hiresRenderer->preDraw(*envs[envIdx], envIdx);
        }

        hiresRenderer->draw(envs);
    }

    void drawOverview()
    {
#ifdef WITH_GUI
        if (!viewer && Viewer::viewerExists) {
            TLOG(INFO) << "Only one viewer per process is supported";
            return;
        }

        if (!viewer) {
            static int argc = 1;
            static const char *argv[] = {"Overview"};
            Magnum::Platform::Application::Arguments fakeArgs{argc, (char **) argv};
            TLOG(INFO) << __FUNCTION__ << " Creating a viewer object";
            viewer = std::make_unique<Viewer>(envs, useVulkan, renderer.get(), fakeArgs);
        }

        viewer->step(vectorEnv->done);
        viewer->mainLoopIteration();  // handle events, update the window, that kind of thing
#else
        TLOG(ERROR) << "VoxelWorld was built without GUI support";
#endif
    }

    py::array_t<uint8_t> getHiresObservation(int envIdx, int agentIdx)
    {
        const uint8_t *obsData = hiresRenderer->getObservation(envIdx, agentIdx);
        return py::array_t<uint8_t>({renderH, renderW, 4}, obsData, py::none{});  // numpy object does not own memory
    }

    float trueObjective(int envIdx, int agentIdx) const
    {
        return vectorEnv->trueObjectives[envIdx][agentIdx];
    }

    std::map<std::string, float> getRewardShaping(int envIdx, int agentIdx)
    {
        return envs[envIdx]->getScenario().getRewardShaping(agentIdx);
    }

    void setRewardShaping(int envIdx, int agentIdx, const std::map<std::string, float> &rewardShaping)
    {
        envs[envIdx]->getScenario().setRewardShaping(agentIdx, rewardShaping);
    }

    /**
     * Explicitly destroy the env and the renderer to avoid doing this when the Python object goes out-of-scope.
     */
    void close()
    {
        if (vectorEnv)
            vectorEnv->close();

#ifdef WITH_GUI
        if (viewer)
            viewer->exit(0);
        viewer.reset();
#endif

        hiresRenderer.reset();
        renderer.reset();
        vectorEnv.reset();

        envs.clear();
    }

private:
    Envs envs;
    int numEnvs, numAgentsPerEnv;
    std::vector<float> rewards;  // to avoid reallocating on every call

    std::unique_ptr<VectorEnv> vectorEnv;
    std::unique_ptr<EnvRenderer> renderer, hiresRenderer;

    Rng rng{std::random_device{}()};

#ifdef WITH_GUI
    std::unique_ptr<Viewer> viewer;
#endif

    bool useVulkan;
    int w, h;
    int renderW = 768, renderH = 432;

    int numSimulationThreads;
};


PYBIND11_MODULE(voxel_env, m)
{
    m.doc() = "voxel env"; // optional module docstring

    m.def("set_voxel_env_log_level", &setVoxelEnvLogLevel, "Voxel Env Log Level (0 to disable all logs, 2 for warnings");

    py::class_<VoxelEnvGym>(m, "VoxelEnvGym")
        .def(py::init<const std::string &, int, int, int, int, int, bool, const FloatParams &>())
        .def("num_agents", &VoxelEnvGym::numAgents)
        .def("action_space_sizes", &VoxelEnvGym::actionSpaceSizes)
        .def("seed", &VoxelEnvGym::seed)
        .def("reset", &VoxelEnvGym::reset)
        .def("set_actions", &VoxelEnvGym::setActions)
        .def("step", &VoxelEnvGym::step)
        .def("is_done", &VoxelEnvGym::isDone)
        .def("get_observation", &VoxelEnvGym::getObservation)
        .def("get_last_rewards", &VoxelEnvGym::getLastRewards)
        .def("true_objective", &VoxelEnvGym::trueObjective)
        .def("set_render_resolution", &VoxelEnvGym::setRenderResolution)
        .def("draw_hires", &VoxelEnvGym::drawHires)
        .def("draw_overview", &VoxelEnvGym::drawOverview)
        .def("get_hires_observation", &VoxelEnvGym::getHiresObservation)
        .def("get_reward_shaping", &VoxelEnvGym::getRewardShaping)
        .def("set_reward_shaping", &VoxelEnvGym::setRewardShaping)
        .def("close", &VoxelEnvGym::close);
}
