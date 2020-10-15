#include <thread>

#include <gtest/gtest.h>

#include <util/tiny_logger.hpp>
#include <util/string_utils.hpp>


using namespace VoxelWorld;

#define TST_LOG(level) LogMessage(level, __FILE__, __LINE__, __FUNCTION__, &testStream)()


namespace
{

// helper functions

void testLog(std::ostringstream &testStream, int threadIdx)
{
    constexpr int n = 5000;
    for (int i = 0; i < n; ++i)
        TST_LOG(INFO) << "thread_" << threadIdx;
}

}


/// Let multiple threads write log messages to the same stream,
/// then parse the output and verify that none of the output is messed up
/// (e.g. no overlapping messages "thread_thread_21, etc.)
TEST(logger, concurrency)
{
    // stream that stores log output during this test, so we can actually verify the results
    std::ostringstream testStream;

    constexpr int numThreads = 10;
    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    for (int i = 0; i < numThreads; ++i)
        threads.emplace_back(testLog, std::ref(testStream), i + 1);

    for (int i = 0; i < numThreads; ++i)
        threads[i].join();

    std::vector<std::string> lines = VoxelWorld::splitString(testStream.str(), "\n");
    int threadIdx = 0;
    for (auto &line : lines)
    {
        const auto msg = splitString(line, " ").back();
        EXPECT_TRUE(startsWith(msg, "thread_"));
        EXPECT_LE(msg.length(), 10);
        std::istringstream(splitString(msg, "_").back()) >> threadIdx;

        EXPECT_GE(threadIdx, 1);
        EXPECT_LE(threadIdx, numThreads);
    }
}
