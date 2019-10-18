cmake_minimum_required(VERSION 3.5)
project(iris_lama VERSION 1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11) # use c++11

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  include(${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
endif()
include_directories(${Eigen_INCLUDE_DIR})

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


add_subdirectory(src)

export(TARGETS iris-lama FILE iris_lamaConfig.cmake)

set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE iris_lama)

