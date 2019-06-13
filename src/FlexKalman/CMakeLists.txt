set(HEADER_LOCATION ${INCLUDE_SOURCE_DIR}/FlexKalman)

set(API
    "${HEADER_LOCATION}/AbsoluteOrientationMeasurement.h"
    "${HEADER_LOCATION}/AbsolutePositionMeasurement.h"
    "${HEADER_LOCATION}/AngularVelocityMeasurement.h"
    "${HEADER_LOCATION}/AugmentedProcessModel.h"
    "${HEADER_LOCATION}/AugmentedState.h"
    "${HEADER_LOCATION}/ClientReportTypesC.h"
    "${HEADER_LOCATION}/ConstantProcess.h"
    "${HEADER_LOCATION}/EigenExtras.h"
    "${HEADER_LOCATION}/EigenFilters.h"
    "${HEADER_LOCATION}/EigenQuatExponentialMap.h"
    "${HEADER_LOCATION}/ExternalQuaternion.h"
    "${HEADER_LOCATION}/ExtractYaw.h"
    "${HEADER_LOCATION}/Finally.h"
    "${HEADER_LOCATION}/FlexibleKalmanBase.h"
    "${HEADER_LOCATION}/FlexibleKalmanCorrect.h"
    "${HEADER_LOCATION}/FlexibleKalmanFilter.h"
    "${HEADER_LOCATION}/FlexibleUnscentedCorrect.h"
    "${HEADER_LOCATION}/MatrixExponentialMap.h"
    "${HEADER_LOCATION}/MiniArgsHandling.h"
    "${HEADER_LOCATION}/OpenCVVersion.h"
    "${HEADER_LOCATION}/OrientationConstantVelocity.h"
    "${HEADER_LOCATION}/OrientationState.h"
    "${HEADER_LOCATION}/PoseConstantVelocity.h"
    "${HEADER_LOCATION}/PoseDampedConstantVelocity.h"
    "${HEADER_LOCATION}/PoseSeparatelyDampedConstantVelocity.h"
    "${HEADER_LOCATION}/PoseStateExponentialMap.h"
    "${HEADER_LOCATION}/PoseState.h"
    "${HEADER_LOCATION}/PureVectorState.h"
    "${HEADER_LOCATION}/SigmaPointGenerator.h"
    "${HEADER_LOCATION}/Tracing.h"
    "${HEADER_LOCATION}/TypeSafeId.h")
source_group(API FILES ${API})

set(SOURCES
    TimeValueC.cpp)

add_library(FlexKalman INTERFACE)
# target_sources(FlexKalman INTERFACE ${API})
target_include_directories(FlexKalman
    INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>)
target_compile_features(FlexKalman
    INTERFACE
    cxx_std_11)


set_property(TARGET FlexKalman PROPERTY INTERFACE_VERSION ${uvbi_VERSION})
set_property(TARGET FlexKalman PROPERTY INTERFACE_FlexKalman_MAJOR_VERSION 0)
set_property(TARGET FlexKalman APPEND PROPERTY
    COMPATIBLE_INTERFACE_STRING INTERFACE_FlexKalman_MAJOR_VERSION
)
###
# Install things properly.
install(TARGETS FlexKalman
    EXPORT uvbiTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Devel
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${API}
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/FlexKalman
    COMPONENT Devel)
