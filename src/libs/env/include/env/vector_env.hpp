#pragma once

#include <atomic>
#include <thread>
#include <condition_variable>

#include <env/env.hpp>
#include <env/env_renderer.hpp>


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

    void step(std::vector<bool> &done);
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

private:
    int numThreads, envsPerThread;
    std::vector<std::thread> backgroundThreads;
    std::vector<Task> currTasks;
    std::condition_variable cvTask;
    std::mutex mutex;
    std::atomic<int> numReady = 0;


};
