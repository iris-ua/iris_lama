
set(lama_SOURCES)
set(lama_SUBDIRS
    # Current diretory
    .
    # Non-Linear Least Squares Optimization
    nlls
    # Sparse-Dense Mapping
    sdm
    # Lossless data compression
    extern/lz4
    extern/zstd
    # (mini) Smoothing and Mapping
    extern/minisam/core
    extern/minisam/utils
    extern/minisam/linear
    extern/minisam/nonlinear
)

foreach(SUBDIR ${lama_SUBDIRS})
    file(GLOB lama_SRC "${SUBDIR}/*.cpp" "${SUBDIR}/*.c")
    list(APPEND lama_SOURCES ${lama_SRC})
endforeach()

add_library(${PROJECT_NAME} ${lama_SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
target_link_libraries(${PROJECT_NAME} pthread)

if(COMPILING_WITH_CATKIN)
    install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    install(DIRECTORY ../include/lama/
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/lama
      PATTERN ".svn" EXCLUDE
    )
elseif(COMPILING_WITH_AMENT)
    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include)
    install(DIRECTORY ../include/lama/
        DESTINATION include/lama
        PATTERN ".svn" EXCLUDE)
else()

    export(PACKAGE iris_lama)

    install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}-config.cmake
        DESTINATION lib/cmake/iris_lama)

    install(TARGETS ${PROJECT_NAME}
        EXPORT   ${PROJECT_NAME}Targets
        ARCHIVE  DESTINATION lib
        LIBRARY  DESTINATION lib
        INCLUDES DESTINATION include)

    install(DIRECTORY ../include/lama/
        DESTINATION include/lama
        PATTERN ".svn" EXCLUDE)

    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        iris_lama-config-version.cmake
        VERSION ${IRIS_LAMA_VERSION}
        COMPATIBILITY AnyNewerVersion)

endif()


