#include <env/vector_env.hpp>

using namespace VoxelWorld;


VectorEnv::VectorEnv(std::vector<std::unique_ptr<Env>> &envs, EnvRenderer &renderer, int numThreads)
: envs(envs)
, renderer(renderer)
, numThreads{numThreads}  // use master threads as one of the threads
{
    const int numEnvs = int(envs.size());
    envsPerThread = (numEnvs / numThreads) + (numEnvs % numThreads != 0);

    currTasks = std::vector<Task>(size_t(numThreads), Task::IDLE);

    for (int i = 1; i < numThreads; ++i) {
        std::thread t{
            [this](int threadIdx) {

                while (true) {
                    std::unique_lock<std::mutex> lock{mutex};

                    while (currTasks[threadIdx] == Task::IDLE)
                        cvTask.wait(lock);

                    const auto task = currTasks[threadIdx];
                    currTasks[threadIdx] = Task::IDLE;
                    lock.unlock();

                    taskFunc(task, threadIdx);
                    ++numReady;

                    if (task == Task::TERMINATE)
                        break;
                }
            }, i
        };

        backgroundThreads.emplace_back(std::move(t));
    }

    done = std::vector<bool>(envs.size());
    trueObjectives = std::vector<float>(envs.size());
}

void VectorEnv::stepEnv(int envIdx)
{
    envs[envIdx]->step();
    renderer.preDraw(*envs[envIdx], envIdx);
}

void VectorEnv::resetEnv(int envIdx)
{
    envs[envIdx]->reset();
}

void VectorEnv::taskFunc(Task task, int threadIdx)
{
    auto func = &VectorEnv::stepEnv;
    if (task == Task::RESET)
        func = &VectorEnv::resetEnv;

    const auto startIdx = threadIdx * envsPerThread;
    const auto endIdx = std::min(startIdx + envsPerThread, int(envs.size()));
    for (int envIdx = startIdx; envIdx < endIdx; ++envIdx)
        (this->*func)(envIdx);
}

void VectorEnv::executeTask(Task task)
{
    numReady = 0;

    std::unique_lock<std::mutex> lock{mutex};
    for (int threadIdx = 1; threadIdx < numThreads; ++threadIdx)
        currTasks[threadIdx] = task;

    cvTask.notify_all();
    lock.unlock();

    taskFunc(task, 0);

    // just waiting on an atomic, this is a bit faster than conditional variable
    // tradeoff - using more CPU cycles?
    while (numReady < numThreads - 1);
}

void VectorEnv::step()
{
    executeTask(Task::STEP);
    renderer.draw(envs);

    // do this in background thread?? (tried, isn't faster really with current postDraw)
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx)
        renderer.postDraw(*envs[envIdx], envIdx);

    // do this in background thread??
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
        if (envs[envIdx]->isDone()) {
            done[envIdx] = true;
            trueObjectives[envIdx] = envs[envIdx]->trueObjective();
            envs[envIdx]->reset();
            renderer.reset(*envs[envIdx], envIdx);
        } else {
            done[envIdx] = false;
        }
    }
}

void VectorEnv::reset()
{
    // reset renderer on the main thread
    for (int envIdx = 0; envIdx < int(envs.size()); ++envIdx) {
        envs[envIdx]->reset();
        renderer.reset(*envs[envIdx], envIdx);
    }
}

void VectorEnv::close()
{
    executeTask(Task::TERMINATE);
    for (auto &t : backgroundThreads)
        t.join();
}
