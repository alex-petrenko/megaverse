#include <env/vector_env.hpp>

using namespace VoxelWorld;

VectorEnv::VectorEnv(std::vector<std::unique_ptr<Env>> &envs,
                     EnvRenderer &renderer, int numThreads)
    : envs(envs), renderer(renderer), numThreads{numThreads}
// use master threads as one of the threads
{
    const int numEnvs = int(envs.size());
    envsPerThread = (numEnvs / numThreads) + (numEnvs % numThreads != 0);

    currTasks = std::vector<Task>(numEnvs, Task::IDLE);
    threadControl = std::vector<Task>(numThreads, Task::IDLE);

    for (int i = 1; i < numThreads; ++i) {
        std::thread t{
            [this, numEnvs](int threadIdx) {
                while (true) {
                    {
                        std::unique_lock<std::mutex> lock{mutex};

                        while (threadControl[threadIdx] == Task::IDLE)
                            cvTask.wait(lock);

                        if (threadControl[threadIdx] == Task::TERMINATE)
                            break;

                        threadControl[threadIdx] = Task::IDLE;
                    }

                    int envIdx = 0;
                    while ((envIdx = nextTaskQueue.fetch_add(1)) < numEnvs) {
                        taskFunc(currTasks[envIdx], envIdx);
                        currTasks[envIdx] = Task::IDLE;
                    }

                    ++numReady;
                }
            },
            i};

        backgroundThreads.emplace_back(std::move(t));
    }

    done = std::vector<bool>(envs.size());
    trueObjectives = std::vector<std::vector<float>>(envs.size());
    for (int envIdx = 0; envIdx < numEnvs; ++envIdx)
        trueObjectives[envIdx] =
            std::vector<float>(envs[envIdx]->getNumAgents());
}

void VectorEnv::stepEnv(int envIdx) {
    envs[envIdx]->step();
    renderer.preDraw(*envs[envIdx], envIdx);
}

void VectorEnv::resetEnv(int envIdx) {
    envs[envIdx]->reset();

    if (renderer.isVulkan()) {
        renderer.reset(*envs[envIdx], envIdx);
        renderer.preDraw(*envs[envIdx], envIdx);
    }
}

void VectorEnv::taskFunc(Task task, int envIdx) {
    if (task == Task::TERMINATE || task == Task::IDLE)
        return;

    auto func = &VectorEnv::stepEnv;
    if (task == Task::RESET)
        func = &VectorEnv::resetEnv;

    (this->*func)(envIdx);
}

void VectorEnv::executeTask(Task task) {
    numReady = 0;
    nextTaskQueue = 0;

    {
        std::unique_lock<std::mutex> lock{mutex};
        for (int threadIdx = 1; threadIdx < numThreads; ++threadIdx)
            threadControl[threadIdx] = task;

        cvTask.notify_all();
    }

    int envIdx = 0;
    while ((envIdx = nextTaskQueue.fetch_add(1)) < int(envs.size())) {
        taskFunc(currTasks[envIdx], envIdx);
        currTasks[envIdx] = Task::IDLE;
    }

    // just waiting on an atomic, this is a bit faster than conditional variable
    // tradeoff - using more CPU cycles?
    while (numReady < numThreads - 1)
        asm volatile("pause" ::: "memory");
}

void VectorEnv::step() {
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx)
        currTasks[envIdx] = Task::STEP;

    executeTask(Task::STEP);

    bool anyDone = false;
    // do this in background thread??
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
        if (envs[envIdx]->isDone()) {
            anyDone = true;
            done[envIdx] = true;
            for (int agentIdx = 0; agentIdx < envs[envIdx]->getNumAgents();
                 ++agentIdx)
                trueObjectives[envIdx][agentIdx] =
                    envs[envIdx]->trueObjective(agentIdx);

            currTasks[envIdx] = Task::RESET;
        } else {
            done[envIdx] = false;
        }
    }

    if (anyDone) {
        executeTask(Task::RESET);

        if (!renderer.isVulkan()) {
            for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
                if (done[envIdx]) {
                    renderer.reset(*envs[envIdx], envIdx);
                    renderer.preDraw(*envs[envIdx], envIdx);
                }
            }
        }
    }

    renderer.draw(envs);
}

void VectorEnv::reset() {
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
        currTasks[envIdx] = Task::RESET;
    }

    executeTask(Task::RESET);

    if (!renderer.isVulkan()) {
        // reset renderer on the main thread for magnum
        for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
            renderer.reset(*envs[envIdx], envIdx);
            renderer.preDraw(*envs[envIdx], envIdx);
        }
    }

    renderer.draw(envs);
}

void VectorEnv::close() {
    executeTask(Task::TERMINATE);
    for (auto &t : backgroundThreads)
        t.join();
}
