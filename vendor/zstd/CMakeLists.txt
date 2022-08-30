
set(zstd_SRC
    zstd/debug.c
    zstd/entropy_common.c
    zstd/error_private.c
    zstd/fse_compress.c
    zstd/fse_decompress.c
    zstd/hist.c
    zstd/huf_compress.c
    zstd/huf_decompress.c
    zstd/pool.c
    zstd/threading.c
    zstd/xxhash.c.inc
    zstd/zstd_common.c
    zstd/zstd_compress.c
    zstd/zstd_ddict.c
    zstd/zstd_decompress.c
    zstd/zstd_double_fast.c
    zstd/zstd_fast.c
    zstd/zstd_lazy.c
    zstd/zstd_ldm.c
    zstd/zstdmt_compress.c
    zstd/zstd_opt.c
)

add_library(zstd OBJECT ${zstd_SRC})
