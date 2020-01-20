cmake_minimum_required(VERSION 3.5)
project(iris_lama VERSION 1.0 LANGUAGES CXX C)

# http://answers.ros.org/question/230877/optionally-build-a-package-with-catkin/
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(COMPILING_WITH_CATKIN 1)
endif()

set(CMAKE_CXX_STANDARD 11) # use c++11

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  include(${PROJECT_SOURCE_DIR}/cmake/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
endif()

# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(COMPILING_WITH_CATKIN)
    message(STATUS "----------------")
    message(STATUS "Catkin detected")
    message(STATUS "----------------")

    find_package(catkin REQUIRED)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
endif()

INCLUDE_DIRECTORIES( include ${Eigen_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} )
add_subdirectory(src)


