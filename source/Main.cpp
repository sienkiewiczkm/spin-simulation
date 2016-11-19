#include <cstdio>
#include <iostream>
#include <string>

#include "SpinApplication.hpp"

int main(int argc, char* argv[])
{
    spin::SpinApplication app;

    try
    {
        app.create();
        app.run();
        app.destroy();
    }
    catch (const std::string& strException)
    {
        std::cerr << "Exception: " << strException << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
