#pragma once

#include <atomic>
#include <thread>
#include <condition_variable>

#include <env/env.hpp>
#include <env/env_renderer.hpp>


namespace VoxelWorld
{

class VectorEnv
{
public:
    enum class Task
    {
        IDLE,
        STEP,
        RESET,
        TERMINATE,
    };

public:
    explicit VectorEnv(Envs &envs, EnvRenderer &renderer, int numThreads);

    void step();

    void reset();

    void close();

private:
    void taskFunc(Task task, int threadIdx);

    void executeTask(Task task);

    void stepEnv(int envIdx);

    void resetEnv(int envIdx);

public:
    std::vector<std::unique_ptr<Env>> &envs;
    EnvRenderer &renderer;

    std::vector<bool> done;
    std::vector<std::vector<float>> trueObjectives;

private:
    int numThreads{}, envsPerThread{};
    std::vector<std::thread> backgroundThreads;
    std::vector<Task> threadControl;
    std::vector<Task> currTasks;
    std::condition_variable cvTask;
    std::mutex mutex;
    std::atomic<int> nextTaskQueue = 0;
    std::atomic<int> numReady = 0;
};

}
