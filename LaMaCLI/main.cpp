#include <iostream>

#include "options.h"

int main(int argc, char* argv[])
{
    auto options = lama::parse_options(argc, argv);
    if (not options) return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
