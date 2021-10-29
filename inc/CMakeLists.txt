set(API
    "FlexKalman/AbsoluteOrientationMeasurement.h"
    "FlexKalman/AbsolutePositionMeasurement.h"
    "FlexKalman/AngularVelocityMeasurement.h"
    "FlexKalman/AugmentedProcessModel.h"
    "FlexKalman/AugmentedState.h"
    "FlexKalman/ClientReportTypesC.h"
    "FlexKalman/ConstantProcess.h"
    "FlexKalman/EigenExtras.h"
    "FlexKalman/EigenFilters.h"
    "FlexKalman/EigenQuatExponentialMap.h"
    "FlexKalman/ExternalQuaternion.h"
    "FlexKalman/ExtractYaw.h"
    "FlexKalman/Finally.h"
    "FlexKalman/FlexibleKalmanBase.h"
    "FlexKalman/FlexibleKalmanCorrect.h"
    "FlexKalman/FlexibleKalmanFilter.h"
    "FlexKalman/FlexibleUnscentedCorrect.h"
    "FlexKalman/MatrixExponentialMap.h"
    "FlexKalman/MiniArgsHandling.h"
    "FlexKalman/OpenCVVersion.h"
    "FlexKalman/OrientationConstantVelocity.h"
    "FlexKalman/OrientationState.h"
    "FlexKalman/PoseConstantVelocity.h"
    "FlexKalman/PoseDampedConstantVelocity.h"
    "FlexKalman/PoseSeparatelyDampedConstantVelocity.h"
    "FlexKalman/PoseStateExponentialMap.h"
    "FlexKalman/PoseState.h"
    "FlexKalman/PureVectorState.h"
    "FlexKalman/SigmaPointGenerator.h"
    "FlexKalman/Tracing.h"
    "FlexKalman/TypeSafeId.h")
source_group(API FILES ${API})

add_library(FlexKalman INTERFACE)
# target_sources(FlexKalman INTERFACE ${API})
target_include_directories(
    FlexKalman INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
                         $<INSTALL_INTERFACE:include>)
target_compile_features(FlexKalman INTERFACE cxx_std_11)

set_property(TARGET FlexKalman PROPERTY INTERFACE_VERSION ${uvbi_VERSION})
set_property(TARGET FlexKalman PROPERTY INTERFACE_FlexKalman_MAJOR_VERSION 0)
set_property(
    TARGET FlexKalman
    APPEND
    PROPERTY COMPATIBLE_INTERFACE_STRING INTERFACE_FlexKalman_MAJOR_VERSION)
#
# Install things properly.
install(
    TARGETS FlexKalman
    EXPORT FlexKalmanTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Devel
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
    INCLUDES
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
    FILES ${API}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/FlexKalman
    COMPONENT Devel)
