
###
# Convenience tool to view the tracker camera using the same pipeline as the tracker plugin.
###
add_executable(uvbi-view-camera ViewTrackingCamera.cpp)
target_link_libraries(uvbi-view-camera PRIVATE uvbi-image-sources opencv_imgcodecs)
