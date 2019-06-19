cmake_minimum_required(VERSION 3.4.0)
project(uvbi VERSION 0.6.0)

set(LOCAL_CMAKE_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake-local")
# Custom CMake modules from https://github.com/rpavlik/cmake-modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" "${LOCAL_CMAKE_MODULE_DIR}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(CTest)
include(GNUInstallDirs)
include(CMakeDependentOption)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

###
# Package Options
###
find_package(OpenCV)
find_package(JsonCpp)

# Only required on Windows - elsewhere, just used for extra tools.
find_package(Boost)

option(BUILD_TOOLS "Build executable tools" ON)

if(WIN32)
    # On Win32, for best experience, enforce the use of the DirectShow capture library.
    # TODO fix this package so it finds things on MSYS2/MinGW64
    # - the lib is in /mingw64/lib
    # - qedit.h is in /mingw64/x86_64-w64-mingw32/include
    if(MSVC)
        # We vendored an MIT-tagged copy from a Microsoft project.
        list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/vendor/qedit)
    endif()
    find_package(DirectShow QUIET)
    if(MINGW OR (DIRECTSHOW_QEDIT_INCLUDE_DIR AND DIRECTSHOW_STRMIIDS_LIBRARY))
        # MinGW may include the header/lib in such a way that the find module
        # can't find them
        set(HAVE_VIDEOTRACKER_DEPS ON)
    else()
        message(STATUS "Skipping the video-based tracker plugin: Requires the 'Microsoft Windows SDK Update for Windows Vista' or older <http://www.microsoft.com/en-us/download/details.aspx?id=23719>")
        if(NOT DIRECTSHOW_QEDIT_INCLUDE_DIR)
            message(STATUS " - DIRECTSHOW_QEDIT_INCLUDE_DIR not found: Couldn't find a qedit.h file declaring ISampleGrabber.")
        endif()
        if(NOT DIRECTSHOW_STRMIIDS_LIBRARY)
            message(STATUS " - DIRECTSHOW_STRMIIDS_LIBRARY not found: Couldn't find the strmiids.lib or libstrmiids.a library.")
        endif()
    endif()
elseif(NOT ANDROID)
    # Non-Windows desktop platforms can use libuvc
    find_package(libuvc)
    if(libuvc_INCLUDE_DIRS AND libuvc_LIBRARIES)
        set(LIBUVC_FOUND ON)
    else()
        set(LIBUVC_FOUND OFF)
    endif()
    find_package(Libusb1)
    if(LIBUSB1_FOUND AND LIBUVC_FOUND)
        set(HAVE_VIDEOTRACKER_DEPS ON)
    else()
        message(STATUS "Video tracker plugin disabled due to missing libuvc or libusb1")
        set(HAVE_VIDEOTRACKER_DEPS OFF)
    endif()
endif()

if(WIN32)
    set(UVBI_EXTRA_CONDITION "AND BOOST_FOUND")
else()
    set(UVBI_EXTRA_CONDITION)
endif()

cmake_dependent_option(BUILD_VIDEOTRACKERSHARED
    "Build the videotrackershared libs"
    ON "OpenCV_FOUND AND JSONCPP_FOUND ${UVBI_EXTRA_CONDITION}" OFF)

cmake_dependent_option(BUILD_UVBI
    "Build the UVBI high-level tracker libs libs"
    ON "BUILD_VIDEOTRACKERSHARED AND HAVE_VIDEOTRACKER_DEPS" OFF)

###
# Building
###
# vendored dependencies
add_subdirectory(vendor)

# Core library
add_subdirectory(src)

if(BUILD_TOOLS)
    add_subdirectory(apps)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

###
# Set Up Package for External Usage
###
if(BUILD_UVBI)
    include(CMakePackageConfigHelpers)
    include(GenerateCompatibilityVersionFile)
    configure_file(cmake/FindJsonCpp.cmake ${CMAKE_CURRENT_BINARY_DIR}/FindJsonCpp.cmake COPYONLY)
    set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/uvbi/cmake")

    generate_compatibility_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/uvbiConfigVersion.cmake"
        VERSION ${uvbi_VERSION}
        COMPATIBILITY AnyNewerVersion
        C_ABI)

    export(TARGETS
        eigen-headers
        FlexKalman
        videotrackershared_core
        videotrackershared_io
        videotrackershared_hdkdata
        uvbi-base
        uvbi-core
        uvbi-image-sources
        uvbi_plugin_parts
        util-headers
        FILE "${CMAKE_CURRENT_BINARY_DIR}/uvbiTargets.cmake"
        NAMESPACE uvbi::)
    export(PACKAGE uvbi)

    configure_file(uvbiConfig.cmake
        "${CMAKE_CURRENT_BINARY_DIR}/uvbiConfig.cmake"
        COPYONLY)

    install(EXPORT uvbiTargets
        FILE
        uvbiTargets.cmake
        NAMESPACE
        uvbi::
        DESTINATION
        ${CONFIG_INSTALL_DIR})

    install(FILES
        uvbiConfig.cmake
        cmake/FindJsonCpp.cmake
        "${CMAKE_CURRENT_BINARY_DIR}/uvbiConfigVersion.cmake"
        DESTINATION
        ${CONFIG_INSTALL_DIR}
        COMPONENT
        Devel)
endif()
