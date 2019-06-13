
# osvr_convert_json(org_osvr_unifiedvideoinertial_json
#     org_osvr_unifiedvideoinertial.json
#     "${CMAKE_CURRENT_BINARY_DIR}/org_osvr_unifiedvideoinertial_json.h")

# Make sure we can link these upcoming static libraries into a shared lib
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DUVBI_USING_EDGE_HOLE_EXTRACTOR)


set(HEADER_LOCATION ${INCLUDE_SOURCE_DIR}/unifiedvideoinertial)
###
# Static library with base utils
###

set(API
    "${HEADER_LOCATION}/APIBaseC.h"
    "${HEADER_LOCATION}/TimeValue.h"
    "${HEADER_LOCATION}/TimeValueC.h"
)
source_group(API FILES ${API})

add_library(uvbi-base STATIC
    TimeValueC.cpp)
target_compile_features(uvbi-base
    PRIVATE
    cxx_std_11)
target_include_directories(uvbi-base
    PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_property(TARGET uvbi-base PROPERTY VERSION ${uvbi_VERSION})
set_property(TARGET uvbi-base PROPERTY SOVERSION 0)
set_property(TARGET uvbi-base PROPERTY INTERFACE_uvbi_MAJOR_VERSION 0)
set_property(TARGET uvbi-base APPEND PROPERTY
    COMPATIBLE_INTERFACE_STRING INTERFACE_uvbi_MAJOR_VERSION
)
###
# Install things properly.
install(TARGETS uvbi-base
    EXPORT uvbiTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Devel
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    FILES
    ${API}
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT
    Devel)

# Image sources
add_subdirectory(ImageSources)

###
# Static library with core of the tracker
###

set(API
    "${HEADER_LOCATION}/Angles.h"
    "${HEADER_LOCATION}/AngVelTools.h"
    "${HEADER_LOCATION}/Assumptions.h"
    "${HEADER_LOCATION}/BeaconIdTypes.h"
    "${HEADER_LOCATION}/BeaconSetupData.h"
    "${HEADER_LOCATION}/BodyIdTypes.h"
    "${HEADER_LOCATION}/CannedIMUMeasurement.h"
    "${HEADER_LOCATION}/ClientReportTypesC.h"
    "${HEADER_LOCATION}/ConfigParams.h"
    "${HEADER_LOCATION}/CrossProductMatrix.h"
    "${HEADER_LOCATION}/CSV.h"
    "${HEADER_LOCATION}/CSVCellGroup.h"
    "${HEADER_LOCATION}/EigenInterop.h"
    "${HEADER_LOCATION}/ImageProcessing.h"
    "${HEADER_LOCATION}/IMUStateMeasurements.h"
    "${HEADER_LOCATION}/MathTypesC.h"
    "${HEADER_LOCATION}/ModelTypes.h"
    "${HEADER_LOCATION}/RangeTransform.h"
    "${HEADER_LOCATION}/SpaceTransformations.h"
    "${HEADER_LOCATION}/TrackedBody.h"
    "${HEADER_LOCATION}/TrackedBodyTarget.h"
    "${HEADER_LOCATION}/TrackingSystem.h"
    "${HEADER_LOCATION}/Types.h"
)
source_group(API FILES ${API})

add_library(uvbi-core STATIC
    ApplyIMUToState.cpp
    ApplyIMUToState.h
    AssignMeasurementsToLeds.h
    BeaconSetupData.cpp
    BodyTargetInterface.h
    Clamp.h
    ConfigParams.cpp
    ForEachTracked.h
    HDKLedIdentifier.cpp
    HDKLedIdentifier.h
    HDKLedIdentifierFactory.cpp
    HDKLedIdentifierFactory.h
    HistoryContainer.h
    ImagePointMeasurement.h
    LED.cpp
    LED.h
    LedIdentifier.cpp
    LedIdentifier.h
    PinholeCameraFlip.h
    PoseEstimator_RANSAC.cpp
    PoseEstimator_RANSAC.h
    PoseEstimator_RANSACKalman.cpp
    PoseEstimator_RANSACKalman.h
    PoseEstimator_SCAATKalman.cpp
    PoseEstimator_SCAATKalman.h
    PoseEstimatorTypes.h
    RoomCalibration.cpp
    RoomCalibration.h
    StateHistory.h
    TrackedBody.cpp
    TrackedBodyIMU.cpp
    TrackedBodyIMU.h
    TrackedBodyTarget.cpp
    TrackingDebugDisplay.cpp
    TrackingDebugDisplay.h
    TrackingSystem_Impl.cpp
    TrackingSystem_Impl.h
    TrackingSystem.cpp
    UsefulQuaternions.h)
target_compile_features(uvbi-core
    PUBLIC
    cxx_std_11)
target_link_libraries(uvbi-core
    PUBLIC
    opencv_core
    opencv_imgproc
    opencv_videoio
    opencv_calib3d
    opencv_highgui
    FlexKalman
    uvbi-base
    $<BUILD_INTERFACE:eigen-headers>
    PRIVATE
    $<BUILD_INTERFACE:videotrackershared_core>
    $<BUILD_INTERFACE:videotrackershared_io>
    $<BUILD_INTERFACE:util-headers>)
target_include_directories(uvbi-core
    PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_definitions(uvbi-core PRIVATE UVBI_CORE)

set_property(TARGET uvbi-core PROPERTY VERSION ${uvbi_VERSION})
set_property(TARGET uvbi-core PROPERTY SOVERSION 0)
set_property(TARGET uvbi-core PROPERTY INTERFACE_uvbi_MAJOR_VERSION 0)
set_property(TARGET uvbi-core APPEND PROPERTY
    COMPATIBLE_INTERFACE_STRING INTERFACE_uvbi_MAJOR_VERSION
)
###
# Install things properly.
install(TARGETS uvbi-core
    EXPORT uvbiTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Devel
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    FILES
    ${API}
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT
    Devel)

# Main plugin, disabled because you can't build an OSVR plugin without OSVR
# osvr_add_plugin(NAME org_osvr_unifiedvideoinertial
#     CPP # indicates we'd like to use the C++ wrapper
#     SOURCES
#     org_osvr_unifiedvideoinertial.cpp
#     "${CMAKE_CURRENT_BINARY_DIR}/org_osvr_unifiedvideoinertial_json.h"
#     AdditionalReports.h
#     ConfigurationParser.h
#     MakeHDKTrackingSystem.h
#     ImageProcessingThread.cpp
#     ImageProcessingThread.h
#     IMUMessage.h
#     ProcessIMUMessage.h
#     ThreadsafeBodyReporting.cpp
#     ThreadsafeBodyReporting.h
#     TrackerThread.cpp
#     TrackerThread.h
#     $<TARGET_OBJECTS:uvbi-hdkdata>
#     ${KALMANFRAMEWORK_VIDEOTRACKERSHARED_SOURCES_IO})

# target_link_libraries(org_osvr_unifiedvideoinertial
#     osvr::osvrAnalysisPluginKit
#     uvbi-core
#     uvbi-image-sources
#     JsonCpp::JsonCpp
#     util-headers
#     folly-headers)

# These are the non-OSVR-dependent files from the plugin:
# everything except the org_osvr_unifiedvideoinertial.cpp
# and the JSON literal
set(API
    "${HEADER_LOCATION}/ConfigurationParser.h"
    "${HEADER_LOCATION}/MakeHDKTrackingSystem.h")
source_group(API FILES ${API})
add_library(uvbi_plugin_parts SHARED
    AdditionalReports.h
    ImageProcessingThread.cpp
    ImageProcessingThread.h
    IMUMessage.h
    ProcessIMUMessage.h
    ThreadsafeBodyReporting.cpp
    ThreadsafeBodyReporting.h
    TrackerThread.cpp
    TrackerThread.h
    ${API})

target_link_libraries(uvbi_plugin_parts
    PUBLIC
    uvbi-core
    uvbi-image-sources
    JsonCpp::JsonCpp
    PRIVATE
    util-headers
    folly-headers
    videotrackershared_io
    videotrackershared_hdkdata
    )
