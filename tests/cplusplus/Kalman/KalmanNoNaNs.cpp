/** @file
    @brief Implementation

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
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
#include "ContentsInvalid.h"
#include "FlexKalman/AbsoluteOrientationMeasurement.h"
#include "FlexKalman/AbsolutePositionMeasurement.h"
#include "FlexKalman/FlexibleKalmanFilter.h"
#include "FlexKalman/PoseConstantVelocity.h"
#include "FlexKalman/PoseDampedConstantVelocity.h"

// Library/third-party includes
#include <catch2/catch.hpp>

// Standard includes
#include <iostream>

using State = flexkalman::pose_externalized_rotation::State;
using AbsoluteOrientationMeasurement =
    flexkalman::AbsoluteOrientationMeasurement<State>;
using AbsolutePositionMeasurement =
    flexkalman::AbsolutePositionMeasurement<State>;

static void dumpState(State const &state, const char msg[], size_t iteration) {
    std::cout << "\n"
              << msg << " (iteration " << iteration << "):\n"
              << state << std::endl;
}

template <typename PM, typename M>
static void runFilterAndCheck(State &state, PM &processModel, M &meas,
                              double dt, size_t iteration) {
    INFO("Iteration " << iteration);
    INFO("prediction step");
    flexkalman::predict(state, processModel, dt);
    dumpState(state, "After prediction", iteration);
    REQUIRE_FALSE(stateContentsInvalid(state));
    REQUIRE_FALSE(covarianceContentsInvalid(state));

    REQUIRE_FALSE(covarianceContentsInvalid(meas.getCovariance(state)));

    INFO("correction step");
    flexkalman::correct(state, processModel, meas);
    dumpState(state, "After correction", iteration);
    REQUIRE_FALSE(stateContentsInvalid(state));
    REQUIRE_FALSE(covarianceContentsInvalid(state));
}

TEMPLATE_TEST_CASE("ProcessModelStability", "",
                   flexkalman::PoseConstantVelocityProcessModel,
                   flexkalman::PoseDampedConstantVelocityProcessModel) {

    using ProcessModel = TestType;
    State state;
    ProcessModel processModel;
    std::size_t iteration = 0;

    dumpState(state, "Initial state", iteration);

    SECTION("IdentityAbsoluteOrientationMeasurement") {
        auto meas = AbsoluteOrientationMeasurement{
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d(0.00001, 0.00001, 0.00001)};
        for (iteration = 0; iteration < 100; ++iteration) {
            runFilterAndCheck(state, processModel, meas, 0.1, iteration);
        }
        // Can't use isApprox to compare to zero vector
        CHECK(state.position().isMuchSmallerThan(0.001));
        /// @todo check that it's roughly identity
    }
    SECTION("IdentityAbsolutePositionMeasurement") {
        auto meas = AbsolutePositionMeasurement{
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.000007)};
        for (iteration = 0; iteration < 100; ++iteration) {
            runFilterAndCheck(state, processModel, meas, 0.1, iteration);
        }
        /// @todo check that it's roughly identity
    }
    SECTION("AbsolutePositionMeasurementXlate111") {
        auto meas = AbsolutePositionMeasurement{
            Eigen::Vector3d::Constant(1), Eigen::Vector3d::Constant(0.000007)};
        for (iteration = 0; iteration < 100; ++iteration) {
            runFilterAndCheck(state, processModel, meas, 0.1, iteration);
        }
        /// @todo check that it's roughly identity orientation, position of 1,
        /// 1, 1
    }
}
