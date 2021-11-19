cmake_minimum_required(VERSION 3.5)
project(iris_lama LANGUAGES CXX C)

set(IRIS_LAMA_VERSION "1.2.0")

# http://answers.ros.org/question/230877/optionally-build-a-package-with-catkin/
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(COMPILING_WITH_CATKIN 1)
elseif( DEFINED ENV{AMENT_PREFIX_PATH})
        set(COMPILING_WITH_AMENT 1)
endif()

set(CMAKE_CXX_STANDARD 14) # use c++14

option(COMPILE_WITH_SSE "Compile with sse4.1 for faster code" ON)
if (COMPILE_WITH_SSE)
    add_compile_options(-msse4.1)
endif()


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
    message(STATUS "---------------")
    message(STATUS "Catkin detected")
    message(STATUS "---------------")

    find_package(catkin REQUIRED)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
elseif(COMPILING_WITH_AMENT)
    message(STATUS "--------------")
    message(STATUS "Ament detected")
    message(STATUS "--------------")

    find_package(ament_cmake REQUIRED)
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
endif()

INCLUDE_DIRECTORIES(include ${Eigen_INCLUDE_DIR})
add_subdirectory(src)

if(COMPILING_WITH_AMENT)
    ament_package()
    # for some reason this variable is set and cmake complains it is not used.
    unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )
endif()


