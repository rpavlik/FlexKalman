cmake_minimum_required(VERSION 3.4.0)
project(uvbi)

set(LOCAL_CMAKE_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake-local")
# Custom CMake modules from https://github.com/rpavlik/cmake-modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" "${LOCAL_CMAKE_MODULE_DIR}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(CTest)

###
# Package Options
###
find_package(OpenCV REQUIRED)
find_package(JsonCpp REQUIRED)

if(WIN32)
    # On Win32, for best experience, enforce the use of the DirectShow capture library.
    # TODO fix this package so it finds things on MSYS2/MinGW64
    # - the lib is in /mingw64/lib
    # - qedit.h is in /mingw64/x86_64-w64-mingw32/include
    find_package(DirectShow QUIET)
    if(MINGW OR (DIRECTSHOW_QEDIT_INCLUDE_DIR AND DIRECTSHOW_STRMIIDS_LIBRARY))
        # MinGW may include the header/lib in such a way that the find module
        # can't find them
        set(BUILD_VIDEOTRACKER_PLUGIN ON)
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
        set(BUILD_VIDEOTRACKER_PLUGIN ON)
    else()
        message(STATUS "HDK video tracker plugin disabled due to missing libuvc or libusb1")
        set(BUILD_VIDEOTRACKER_PLUGIN OFF)
    endif()
endif()

###
# Building
###
# vendored dependencies
add_subdirectory(vendor)

# Core library
add_subdirectory(src/KalmanFramework)

add_subdirectory(plugins)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
