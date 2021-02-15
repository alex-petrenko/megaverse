#include <env/vector_env.hpp>

using namespace VoxelWorld;

VectorEnv::VectorEnv(std::vector<std::unique_ptr<Env>> &envs,
                     EnvRenderer &renderer, int numThreads)
    : envs(envs), renderer(renderer), numThreads{numThreads}
// use master threads as one of the threads
{
    const int numEnvs = int(envs.size());
    envsPerThread = (numEnvs / numThreads) + (numEnvs % numThreads != 0);

    currTasks = std::vector<Task>(numThreads, Task::IDLE);

    for (int i = 1; i < numThreads; ++i) {
        std::thread t{[this, numEnvs](int threadIdx) {
                          while (true) {
                              Task task;
                              {
                                  std::unique_lock<std::mutex> lock{mutex};

                                  cvTask.wait(lock, [this, &threadIdx] {
                                      return currTasks[threadIdx] != Task::IDLE;
                                  });
                                  task = currTasks[threadIdx];

                                  currTasks[threadIdx] = Task::IDLE;
                              }

                              int envIdx = 0;
                              while ((envIdx = nextTaskQueue.fetch_add(
                                          1, std::memory_order_acq_rel)) <
                                     numEnvs) {
                                  taskFunc(task, envIdx);
                              }

                              numReady.fetch_add(1, std::memory_order_acq_rel);

                              if (task == Task::TERMINATE)
                                  break;
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

    if (!envs[envIdx]->isDone())
        renderer.preDraw(*envs[envIdx], envIdx);
}

void VectorEnv::resetEnv(int envIdx) {
    envs[envIdx]->reset();

    // The vulkan renderer is fine with being reset in parallel
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

    if (task == Task::STEP) {
        if (envs[envIdx]->isDone()) {
            done[envIdx] = true;
            for (int agentIdx = 0; agentIdx < envs[envIdx]->getNumAgents();
                 ++agentIdx)
                trueObjectives[envIdx][agentIdx] =
                    envs[envIdx]->trueObjective(agentIdx);

            resetEnv(envIdx);
        } else {
            done[envIdx] = false;
        }
    }
}

void VectorEnv::executeTask(Task task) {
    numReady.store(0, std::memory_order_relaxed);
    nextTaskQueue.store(0, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    {
        std::unique_lock<std::mutex> lock{mutex};
        for (int threadIdx = 1; threadIdx < numThreads; ++threadIdx)
            currTasks[threadIdx] = task;
    }
    cvTask.notify_all();

    int envIdx = 0;
    while ((envIdx = nextTaskQueue.fetch_add(1, std::memory_order_acq_rel)) <
           int(envs.size())) {
        taskFunc(task, envIdx);
    }

    // just waiting on an atomic, this is a bit faster than conditional variable
    // tradeoff - using more CPU cycles?
    while (numReady.load(std::memory_order_acquire) < numThreads - 1)
        asm volatile("pause" ::: "memory");

    std::atomic_thread_fence(std::memory_order_acquire);
}

void VectorEnv::step() {
    executeTask(Task::STEP);

    // The OpenGL renderer isn't okay with being reset in parallel
    if (!renderer.isVulkan()) {
        for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
            if (done[envIdx]) {
                renderer.reset(*envs[envIdx], envIdx);
                renderer.preDraw(*envs[envIdx], envIdx);
            }
        }
    }

    renderer.draw(envs);
}

void VectorEnv::reset() {
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
