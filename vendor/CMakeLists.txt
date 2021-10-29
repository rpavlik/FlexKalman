if(BUILD_TESTING)
    # Will search for a system install first.
    add_subdirectory(catch-single-header-2.13.7)
endif()

# Interface target for Eigen
add_library(eigen-headers INTERFACE)
target_compile_definitions(eigen-headers INTERFACE EIGEN_MPL2_ONLY)
find_package(Eigen3 QUIET)
if(EIGEN3_FOUND)
    target_include_directories(eigen-headers INTERFACE "${EIGEN3_INCLUDE_DIR}")
else()
    # Bundled 3.3.7
    target_include_directories(eigen-headers
                               INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
endif()
