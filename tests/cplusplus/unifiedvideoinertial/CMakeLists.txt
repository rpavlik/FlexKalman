# SPDX-License-Identifier: Apache-2.0

###
# Synthetic-data verification of usage of room calibration in IMU code and IMU filtering
###
add_executable(uvbi-test-imu
    TestIMU_Common.h
    TestIMU_EKF.cpp
    TestIMU_UKF.cpp)
target_link_libraries(uvbi-test-imu PRIVATE uvbi-core kf-catch2-main)
add_test(NAME TestIMU COMMAND uvbi-test-imu)
