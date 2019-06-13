
add_library(kf-catch2-main STATIC CatchMain.cpp)
target_link_libraries(kf-catch2-main PUBLIC kf-catch2-interface)
target_compile_definitions(kf-catch2-main PUBLIC CATCH_CONFIG_CONSOLE_WIDTH=120)

add_subdirectory(Kalman)
add_subdirectory(Util)
if(BUILD_UVBI)
    add_subdirectory(unifiedvideoinertial)
endif()
