#include <util/argparse.hpp>

#include <env/scenario.hpp>

#include "viewer_args.hpp"


argparse::ArgumentParser Megaverse::viewerStandardArgParse(const std::string &appName) {
    argparse::ArgumentParser parser(appName);

    parser.add_argument("-l", "--list_scenarios")
        .help("list registered scenario names")
        .action([&](const auto &) {
            auto s = Scenario::registeredScenarios();
            TLOG(INFO) << "Scenarios: " << s;
        })
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--scenario")
        .help("name of the scenario to run")
        .default_value(std::string{"ObstaclesEasy"});
    parser.add_argument("--num_agents")
        .help("size of the team")
        .default_value(2)
        .scan<'i', int>();

    constexpr bool isApple =
#if defined(CORRADE_TARGET_APPLE)
        true;
#else
        false;
#endif

    parser.add_argument("--use_opengl")
        .help("Whether to use OpenGL renderer instead of fast Vulkan renderer (currently Vulkan is only supported in Linux)")
        .default_value(isApple)
        .implicit_value(true);

    return parser;
}

void Megaverse::parseArgs(argparse::ArgumentParser &p, int argc, char** argv)
{
    try {
        p.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << p;
        std::exit(EXIT_FAILURE);
    }
}
