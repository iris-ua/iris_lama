#include <iostream>
#include <cxxopts.hpp>

#include "options.h"

std::optional<lama::CLIOptions> lama::parse_options(int argc, char* argv[])
{
    CLIOptions co;
    cxxopts::Options options("lama-cli");
    options.add_options()
        ("h,help",  "this help")
        // ("g,graph", "do graph SLAM", cxxopts::value<bool>(co.graph_slam))
        ("i,input", "input dataset", cxxopts::value<std::string>(co.dataset))
        ;

    options.add_options("common")
        ("odom_frame", "odometry frame",
            cxxopts::value<std::string>(co.odom_frame_id)->default_value("") )
        ("base_frame", "robot's base frame",
            cxxopts::value<std::string>(co.base_frame_id)->default_value("base_link") )
        ("scan_topic", "the name of the scan topic",
            cxxopts::value<std::string>(co.scan_topic)->default_value("") )
        ;

    cxxopts::ParseResult result;
    try {
        result = options.parse(argc, argv);
    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "error parsing options: " << e.what() << std::endl;
        return std::nullopt;
    }

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    if (not result.count("input")) {
        std::cerr << "no input file: try --help for usage" << std::endl;
        return std::nullopt;
    }

    return co;
}
