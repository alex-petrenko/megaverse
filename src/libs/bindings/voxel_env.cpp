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
    VoxelEnvGym(int w, int h)
    : w{w}, h{h}
    {
        env = std::make_unique<Env>(42);
        renderer = std::make_unique<MagnumEnvRenderer>(*env, w, h);
    }

    int numAgents() const
    {
        return env->numAgents;
    }

    void reset()
    {
        env->reset();
        renderer->reset(*env);
    }

    void setAction(int agentIdx, int actionIdx)
    {
        env->setAction(agentIdx, Action(1 << actionIdx));
    }

    bool step()
    {
        auto done = env->step();
        renderer->draw(*env);
        return done;
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
            for (int i = 0; i < env->numAgents; ++i) {
                const auto wname = std::to_string(i);
                cv::namedWindow(wname);
                cv::moveWindow(wname, int(w * i * 1.1), 0);
            }
            windowsInitialized = true;
        }

        for (int i = 0; i < env->numAgents; ++i) {
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
        .def(py::init<int, int>())
        .def("numAgents", &VoxelEnvGym::numAgents)
        .def("reset", &VoxelEnvGym::reset)
        .def("setAction", &VoxelEnvGym::setAction)
        .def("step", &VoxelEnvGym::step)
        .def("getObservation", &VoxelEnvGym::getObservation)
        .def("close", &VoxelEnvGym::close);
}