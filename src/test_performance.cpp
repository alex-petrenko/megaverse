#include <Corrade/Utility/DebugStl.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Platform/WindowlessEglApplication.h>

using namespace Magnum;

class MyApplication: public Platform::WindowlessApplication {
public:
    MyApplication(const Arguments& arguments);

    int exec() override;
};

MyApplication::MyApplication(const Arguments& arguments):
        Platform::WindowlessApplication{arguments} {}

int MyApplication::exec() {
    Debug{} << "OpenGL version:" << GL::Context::current().versionString();
    Debug{} << "OpenGL renderer:" << GL::Context::current().rendererString();

    /* Exit with success */
    return 0;
}

/* main() function implementation */
MAGNUM_WINDOWLESSAPPLICATION_MAIN(MyApplication)