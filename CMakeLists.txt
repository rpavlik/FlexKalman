cmake_minimum_required(VERSION 3.4.0)
project(FlexKalman VERSION 0.6.0)

set(LOCAL_CMAKE_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake-local")
# Custom CMake modules from https://github.com/rpavlik/cmake-modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
     "${LOCAL_CMAKE_MODULE_DIR}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(CTest)
include(GNUInstallDirs)
include(CMakeDependentOption)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

###
# Package Options
###

# Only required on Windows - elsewhere, just used for extra tools.
find_package(Boost)

###
# Building
###
# vendored dependencies
add_subdirectory(vendor)

# Core library
add_subdirectory(inc)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

###
# Set Up Package for External Usage
###
include(CMakePackageConfigHelpers)
include(GenerateCompatibilityVersionFile)
set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/FlexKalman/cmake")
configure_file(cmake-local/FindEigen3.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/FindEigen3.cmake COPYONLY)

generate_compatibility_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/FlexKalmanConfigVersion.cmake" VERSION
    ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion C_ABI)

export(
    TARGETS eigen-headers FlexKalman
    FILE "${CMAKE_CURRENT_BINARY_DIR}/FlexKalmanTargets.cmake"
    NAMESPACE FlexKalman::)
export(PACKAGE FlexKalman)

configure_file(FlexKalmanConfig.cmake.in
               "${CMAKE_CURRENT_BINARY_DIR}/FlexKalmanConfig.cmake" COPYONLY)

install(
    EXPORT FlexKalmanTargets
    FILE FlexKalmanTargets.cmake
    NAMESPACE FlexKalman::
    DESTINATION ${CONFIG_INSTALL_DIR})

install(
    FILES FlexKalmanConfig.cmake cmake/FindEigen3.cmake
          "${CMAKE_CURRENT_BINARY_DIR}/FlexKalmanConfigVersion.cmake"
    DESTINATION ${CONFIG_INSTALL_DIR}
    COMPONENT Devel)
