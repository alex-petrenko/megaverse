#include <env/env.hpp>
#include <v4r_rendering/v4r_env_renderer.hpp>
#include <magnum_rendering/magnum_env_renderer.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <util/tiny_logger.hpp>

namespace py = pybind11;



void setVoxelEnvLogLevel(int level)
{
    setLogLevel(LogLevel(level));
}


class VoxelEnvGym
{
public:
    VoxelEnvGym(int w, int h, int numAgents, float verticalLookLimit, bool useVulkan)
    : useVulkan{useVulkan}, w{w}, h{h}
    {
        env = std::make_unique<Env>(numAgents, verticalLookLimit);
    }

    void seed(int seedValue)
    {
        env->seed(seedValue);
    }

    int numAgents() const
    {
        return env->getNumAgents();
    }

    void reset()
    {
        env->reset();

        if (!renderer) {
            if (useVulkan)
                renderer = std::make_unique<V4REnvRenderer>(*env, w, h);
            else
                renderer = std::make_unique<MagnumEnvRenderer>(*env, w, h);
        }

        renderer->reset(*env);
        renderer->draw(*env);

        if (hiresRenderer)
            hiresRenderer->reset(*env);
    }

    void setAction(int agentIdx, int actionIdx)
    {
        env->setAction(agentIdx, Action(1 << actionIdx));
    }

    void setActionMask(int agentIdx, int actionMask)
    {
        env->setAction(agentIdx, Action(actionMask));
    }

    bool step()
    {
        auto done = env->step();
        renderer->draw(*env);
        return done;
    }

    float getLastReward(int agentIdx)
    {
        return env->getLastReward(agentIdx);
    }

    py::array_t<uint8_t> getObservation(int agentIdx)
    {
        const uint8_t *obsData = renderer->getObservation(agentIdx);
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
                hiresRenderer = std::make_unique<V4REnvRenderer>(*env, renderW, renderH);
            else
                hiresRenderer = std::make_unique<MagnumEnvRenderer>(*env, renderW, renderH);

            hiresRenderer->reset(*env);
        }

        hiresRenderer->draw(*env);
    }

    py::array_t<uint8_t> getHiresObservation(int agentIdx)
    {
        const uint8_t *obsData = hiresRenderer->getObservation(agentIdx);
        return py::array_t<uint8_t>({renderH, renderW, 4}, obsData, py::none{});  // numpy object does not own memory
    }

    bool isLevelCompleted() const
    {
        return env->isLevelCompleted();
    }

    float trueObjective() const
    {
        return env->trueObjective();
    }

    /**
     * Explicitly destroy the env and the renderer to avoid doing this when the Python object goes out-of-scope.
     */
    void close()
    {
        hiresRenderer.reset();
        renderer.reset();
        env.reset();
    }

private:
    std::unique_ptr<Env> env;
    std::unique_ptr<EnvRenderer> renderer, hiresRenderer;

    bool useVulkan;
    int w, h;
    int renderW = 768, renderH = 432;
};



PYBIND11_MODULE(voxel_env, m)
{
    m.doc() = "voxel env"; // optional module docstring

    m.def("set_voxel_env_log_level", &setVoxelEnvLogLevel, "Voxel Env Log Level (0 to disable all logs, 2 for warnings");

    py::class_<VoxelEnvGym>(m, "VoxelEnvGym")
        .def(py::init<int, int, int, float, bool>())
        .def("num_agents", &VoxelEnvGym::numAgents)
        .def("seed", &VoxelEnvGym::seed)
        .def("reset", &VoxelEnvGym::reset)
        .def("set_action", &VoxelEnvGym::setAction)
        .def("set_action_mask", &VoxelEnvGym::setActionMask)
        .def("step", &VoxelEnvGym::step)
        .def("get_observation", &VoxelEnvGym::getObservation)
        .def("get_last_reward", &VoxelEnvGym::getLastReward)
        .def("is_level_completed", &VoxelEnvGym::isLevelCompleted)
        .def("true_objective", &VoxelEnvGym::trueObjective)
        .def("set_render_resolution", &VoxelEnvGym::setRenderResolution)
        .def("draw_hires", &VoxelEnvGym::drawHires)
        .def("get_hires_observation", &VoxelEnvGym::getHiresObservation)
        .def("close", &VoxelEnvGym::close);
}