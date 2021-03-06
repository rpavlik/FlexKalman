/** @file
    @brief Implementation

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// SPDX-License-Identifer: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
template <typename T>
inline void dumpKalmanDebugOuput(const char name[], const char expr[],
                                 T const &value) {
    std::cout << "\n(Kalman Debug Output) " << name << " [" << expr << "]:\n"
              << value << std::endl;
}

#define FLEXKALMAN_DEBUG_OUTPUT(Name, Value)                                   \
    dumpKalmanDebugOuput(Name, #Value, Value)

// Internal Includes
#include "FlexKalman/AbsoluteOrientationMeasurement.h"
#include "FlexKalman/FlexibleKalmanFilter.h"
#include "FlexKalman/PoseConstantVelocity.h"

#include "ContentsInvalid.h"

// Library/third-party includes
// - none

// Standard includes
#include <iostream>

int main() {
    using ProcessModel = flexkalman::PoseConstantVelocityProcessModel;
    using State = flexkalman::pose_externalized_rotation::State;
    using Measurement = flexkalman::AbsoluteOrientationEKFMeasurement<State>;
    State state;
    ProcessModel processModel;
    std::cout << "Initial state:" << std::endl;
    std::cout << state << std::endl;
    {
        auto meas = Measurement{Eigen::Quaterniond::Identity(),
                                Eigen::Vector3d(0.00001, 0.00001, 0.00001)};
        std::cout << "Measurement covariance:\n"
                  << meas.getCovariance(state) << std::endl;

        for (int i = 0; i < 100; ++i) {
            flexkalman::predict(state, processModel, 0.1);

            std::cout << "\nAfter prediction (iteration " << i << "):\n"
                      << state << std::endl;
            if (stateContentsInvalid(state)) {
                std::cout << "ERROR: Detected invalid state contents after "
                             "prediction step of iteration "
                          << i << std::endl;
                return -1;
            }
            if (covarianceContentsInvalid(state)) {
                std::cout << "ERROR: Detected invalid covariance contents "
                             "after prediction step of iteration "
                          << i << std::endl;
                return -1;
            }
            flexkalman::correct(state, processModel, meas);
            std::cout << "\nAfter correction (iteration " << i << "):\n"
                      << state << std::endl;
            if (stateContentsInvalid(state)) {
                std::cout << "ERROR: Detected invalid state contents after "
                             "correction step of iteration "
                          << i << std::endl;
                return -1;
            }
            if (covarianceContentsInvalid(state)) {
                std::cout << "ERROR: Detected invalid covariance contents "
                             "after correction step of iteration "
                          << i << std::endl;
                return -1;
            }
        }
    }
    std::cout << "SUCCESS: ran till completion of configured number of "
                 "iterations without denormalizing in some way."
              << std::endl;
    return 0;
}
