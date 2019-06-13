# SPDX-License-Identifier: Apache-2.0

add_executable(blob_extraction_demo
    BlobExtractionDemo.cpp)
target_link_libraries(blob_extraction_demo
    PUBLIC
    videotrackershared_core
    videotrackershared_io
    JsonCpp::JsonCpp
    opencv_highgui
    opencv_imgproc
    opencv_videoio
    opencv_imgcodecs)
target_compile_definitions(blob_extraction_demo
    PRIVATE
    UVBI_USING_EDGE_HOLE_EXTRACTOR)
