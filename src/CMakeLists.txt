
set(INCLUDE_SOURCE_DIR ${PROJECT_SOURCE_DIR}/inc)

add_subdirectory(FlexKalman)

# if(WIN32)
#     add_subdirectory(DirectShowCameraLib)
# endif()

if(BUILD_VIDEOTRACKERSHARED)
    add_subdirectory(videotrackershared)
endif()

if(BUILD_UVBI)
    add_subdirectory(unifiedvideoinertial)
endif()
