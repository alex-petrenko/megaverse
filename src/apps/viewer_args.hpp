#include <util/argparse.hpp>

namespace Megaverse
{

argparse::ArgumentParser viewerStandardArgParse(const std::string &appName);
void parseArgs(argparse::ArgumentParser &p, int argc, char** argv);

}