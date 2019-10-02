cmake_minimum_required(VERSION 3.10)
project(iris_lama VERSION 1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11) # use c++11

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(src)

export(TARGETS iris-lama FILE iris_lamaConfig.cmake)

set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE iris_lama)

