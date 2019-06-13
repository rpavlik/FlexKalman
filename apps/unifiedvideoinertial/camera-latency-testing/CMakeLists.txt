
###
# Utility to measure capture latency of the IR camera, USB stack, etc.
###
add_executable(uvbi-measure-camera-latency MeasureTrackingCameraLatency.cpp)
target_link_libraries(uvbi-measure-camera-latency PRIVATE uvbi-image-sources vendored-vrpn)
