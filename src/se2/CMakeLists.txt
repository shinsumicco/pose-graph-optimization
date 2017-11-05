file(GLOB_RECURSE SOURCES ./*.cc)
file(GLOB_RECURSE HEADERS ./*.h)

add_executable(se2_optimize
        ${SOURCES}
        ${HEADERS})
target_link_libraries(se2_optimize
        ${CERES_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${CXSPARSE_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        ${GFLAGS_LIBRARY}
        )
