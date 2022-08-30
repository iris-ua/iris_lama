/**
 * @file  config.h
 * @brief configurations import from CMake
 * @author Jing Dong
 * @date  Sep 21, 2018
 */

#pragma once

// enable multi-threading
//#define MINISAM_WITH_MULTI_THREADS

// number of threads to use if enable multi-threading
//#define MINISAM_WITH_MULTI_THREADS_NUM 4

// enable internal timing
#define MINISAM_WITH_INTERNAL_TIMING

// Whether use Sophus
#define MINISAM_USE_SOPHUS

// Whether use CUDA cuSOLVER
// define MINISAM_USE_CUSOLVER

// Whether use SuiteSparse CHOLMOD solver
// #define MINISAM_USE_CHOLMOD

// Whether use SuiteSparse SPQR solver
// #define MINISAM_USE_SPQR
