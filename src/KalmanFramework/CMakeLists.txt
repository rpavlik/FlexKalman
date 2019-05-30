set(HEADER_LOCATION "${PROJECT_SOURCE_DIR}/inc/KalmanFramework")

set(API
    "${HEADER_LOCATION}/AbsoluteOrientationMeasurement.h"
    "${HEADER_LOCATION}/AbsolutePositionMeasurement.h"
    "${HEADER_LOCATION}/AngularVelocityMeasurement.h"
    "${HEADER_LOCATION}/AugmentedProcessModel.h"
    "${HEADER_LOCATION}/AugmentedState.h"
    "${HEADER_LOCATION}/ConstantProcess.h"
    "${HEADER_LOCATION}/ExternalQuaternion.h"
    "${HEADER_LOCATION}/FlexibleKalmanBase.h"
    "${HEADER_LOCATION}/FlexibleKalmanCorrect.h"
    "${HEADER_LOCATION}/FlexibleKalmanFilter.h"
    "${HEADER_LOCATION}/OrientationConstantVelocity.h"
    "${HEADER_LOCATION}/OrientationState.h"
    "${HEADER_LOCATION}/PoseConstantVelocity.h"
    "${HEADER_LOCATION}/PoseDampedConstantVelocity.h"
    "${HEADER_LOCATION}/PoseState.h"
    "${HEADER_LOCATION}/PureVectorState.h")
list(APPEND API "${CMAKE_CURRENT_BINARY_DIR}/Export.h")
source_group(API FILES ${API})
set(SOURCES
    TimeValueC.cpp)

add_library(KalmanFramework STATIC ${API} ${SOURCES})
target_include_directories(KalmanFramework
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>)
target_compile_features(KalmanFramework
    PUBLIC
    cxx_std_11)

set(EXPORT_BASENAME KALMANFRAMEWORK)
configure_file(Export.h.in
    Export.h
    @ONLY NEWLINE_STYLE LF)

if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(KalmanFramework PUBLIC KALMANFRAMEWORK_STATIC_DEFINE)
endif()
###
# Install things properly.
install(TARGETS KalmanFramework
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Devel
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${API}
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/KalmanFramework
    COMPONENT Devel)
