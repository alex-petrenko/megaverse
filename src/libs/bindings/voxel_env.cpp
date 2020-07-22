#include <env/env.hpp>
#include <magnum_rendering/magnum_env_renderer.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace py = pybind11;


class VoxelEnvGym
{
public:
    VoxelEnvGym(int w, int h, int numAgents)
    : w{w}, h{h}
    {
        env = std::make_unique<Env>(numAgents);
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

        if (!renderer)
            renderer = std::make_unique<MagnumEnvRenderer>(*env, w, h);
        renderer->reset(*env);
        renderer->draw(*env);
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

    void render()
    {
        // TODO: remove

        if (!windowsInitialized) {
            for (int i = 0; i < env->getNumAgents(); ++i) {
                const auto wname = std::to_string(i);
                cv::namedWindow(wname);
                cv::moveWindow(wname, int(w * i * 1.1), 0);
            }
            windowsInitialized = true;
        }

        for (int i = 0; i < env->getNumAgents(); ++i) {
            const uint8_t *obsData = renderer->getObservation(i);

            cv::Mat mat(h, w, CV_8UC4, (char *) obsData);
            cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
            cv::flip(mat, mat, 0);
            cv::imshow(std::to_string(i), mat);
        }

        cv::waitKey(1);
    }

    /**
     * Explicitly destroy the env and the renderer to avoid doing this when the Python object goes out-of-scope.
     */
    void close()
    {
        renderer.reset();
        env.reset();
    }

private:
    std::unique_ptr<Env> env;
    std::unique_ptr<MagnumEnvRenderer> renderer;

    int w, h;
    bool windowsInitialized = false;
};



PYBIND11_MODULE(voxel_env, m)
{
    m.doc() = "voxel env"; // optional module docstring

    py::class_<VoxelEnvGym>(m, "VoxelEnvGym")
        .def(py::init<int, int, int>())
        .def("num_agents", &VoxelEnvGym::numAgents)
        .def("seed", &VoxelEnvGym::seed)
        .def("reset", &VoxelEnvGym::reset)
        .def("set_action", &VoxelEnvGym::setAction)
        .def("set_action_mask", &VoxelEnvGym::setActionMask)
        .def("step", &VoxelEnvGym::step)
        .def("get_observation", &VoxelEnvGym::getObservation)
        .def("get_last_reward", &VoxelEnvGym::getLastReward)
        .def("close", &VoxelEnvGym::close);
}