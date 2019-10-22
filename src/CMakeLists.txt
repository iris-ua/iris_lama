
set(lama_SOURCES
    pose2d.cpp
    pose3d.cpp
    print.cpp
    buffer_compressor.cpp
    random.cpp
    image_io.cpp
    thread_pool.cpp
    # Non-Linear Least Squares Optimization
    nlls/solver.cpp
    nlls/robust_cost.cpp
    nlls/gauss_newton.cpp
    nlls/levenberg_marquardt.cpp
    # Sparse-Dense Mapping
    sdm/container.cpp
    sdm/map.cpp
    sdm/dynamic_distance_map.cpp
    sdm/simple_occupancy_map.cpp
    sdm/frequency_occupancy_map.cpp
    sdm/probabilistic_occupancy_map.cpp
    sdm/export.cpp
    # Lossless data compression
    extern/lz4.c
    extern/zstd/entropy_common.c
    extern/zstd/fse_decompress.c
    extern/zstd/threading.c
    extern/zstd/pool.c
    extern/zstd/zstd_common.c
    extern/zstd/error_private.c
    extern/zstd/debug.c
    extern/zstd/fse_compress.c
    extern/zstd/huf_compress.c
    extern/zstd/zstd_compress.c
    extern/zstd/zstdmt_compress.c
    extern/zstd/zstd_fast.c
    extern/zstd/zstd_double_fast.c
    extern/zstd/zstd_lazy.c
    extern/zstd/zstd_opt.c
    extern/zstd/zstd_ldm.c
    extern/zstd/hist.c
    extern/zstd/huf_decompress.c
    extern/zstd/zstd_decompress.c
    extern/zstd/zstd_ddict.c
    # Localization and Mapping
    match_surface_2d.cpp
    loc2d.cpp
    slam2d.cpp
    pf_slam2d.cpp
)

add_library(iris-lama STATIC ${lama_SOURCES})

target_include_directories(iris-lama PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_compile_options(iris-lama PRIVATE -Wall)


