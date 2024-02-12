cmake_minimum_required(VERSION 3.10.2)
project(lama-cli)

set(lama_cli_SRC
    main.cpp
    options.cpp
)

add_executable(lama-cli ${lama_cli_SRC})
target_link_libraries(lama-cli cxxopts)
