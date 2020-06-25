#include <util/test_utils.hpp>
#include <util/tiny_logger.hpp>


namespace
{

const char *testFolderEnvVar = "TEST_DATA_PATH_4DVIDEO";

}


std::string getTestDataFolder()
{
    static std::string testDataFolder;
    if (!testDataFolder.empty())
        return testDataFolder;

    const char *testFolderFromEnv = getenv(testFolderEnvVar);
    if (testFolderFromEnv)
    {
        testDataFolder = testFolderFromEnv;
        TLOG(INFO) << "test folder obtained from env: " << testDataFolder;
        return testDataFolder;
    }

    TLOG(ERROR) << "test data folder not found!";
    throw std::runtime_error("Test data folder not found!");
}
