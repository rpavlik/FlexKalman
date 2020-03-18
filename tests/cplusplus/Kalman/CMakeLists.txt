#KalmanQuatNoNaNs
foreach(test KalmanNoNaNs KalmanCombinedNoNaNs  KalmanExpNoNaNs KalmanAbsOrient SmallAngle)
    add_executable(Test${test}
        ${test}.cpp)
    target_link_libraries(Test${test} FlexKalman eigen-headers kf-catch2-main)
    add_test(NAME Test${test} COMMAND Test${test})
endforeach()

add_executable(Kalman_ManualTest ContentsInvalid.h ManualTest.cpp)
target_link_libraries(Kalman_ManualTest FlexKalman eigen-headers)

add_executable(ManualDump ManualDump.cpp)
target_link_libraries(ManualDump FlexKalman eigen-headers)


add_executable(DataDriven DataDrivenMain.cpp DataDriven.cpp)
target_link_libraries(DataDriven FlexKalman eigen-headers pthread CLI11-headers csv-headers)
