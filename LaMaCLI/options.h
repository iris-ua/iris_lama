
#pragma once

#include <string>
#include <optional>

namespace lama {

struct CLIOptions {
    std::string dataset;

    std::string odom_frame_id;
    std::string base_frame_id;
    std::string scan_topic;
};

std::optional<CLIOptions> parse_options(int argc, char* argv[]);

}// namespace lama
