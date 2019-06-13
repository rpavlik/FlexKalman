foreach(testname  QuatExpMap)
    add_executable(${testname}
        ${testname}.cpp)
    target_link_libraries(${testname} kf-catch2-main)
    target_link_libraries(${testname} FlexKalman)
    add_test(NAME ${testname} COMMAND ${testname})
endforeach()

target_link_libraries(QuatExpMap eigen-headers)
