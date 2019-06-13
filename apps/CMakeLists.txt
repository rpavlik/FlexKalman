# SPDX-License-Identifier: BSL-1.0

if(BUILD_VIDEOTRACKERSHARED)
    add_subdirectory(videotrackershared)
endif()

if(BUILD_UVBI)
    add_subdirectory(unifiedvideoinertial)
endif()
