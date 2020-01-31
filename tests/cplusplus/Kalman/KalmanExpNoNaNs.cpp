/** @file
    @brief Implementation

    @date 2015-2020

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2019-2020 Collabora, Ltd.
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

#if 0
template <typename T>
inline void dumpKalmanDebugOuput(const char name[], const char expr[],
                                 T const &value) {
    std::cout << "\n(Kalman Debug Output) " << name << " [" << expr << "]:\n"
              << value << std::endl;
}

#define FLEXKALMAN_DEBUG_OUTPUT(Name, Value)                                   \
    dumpKalmanDebugOuput(Name, #Value, Value)
#endif

// Internal Includes
#include "FlexKalman/AbsoluteOrientationMeasurement.h"
#include "FlexKalman/AbsolutePositionMeasurement.h"
#include "FlexKalman/FlexibleKalmanFilter.h"
#include "FlexKalman/FlexibleUnscentedCorrect.h"
#include "FlexKalman/PoseStateExponentialMap.h"

#include <catch2/catch.hpp>

#include "ContentsInvalid.h"

using State = flexkalman::pose_exp_map::State;
using flexkalman::AbsoluteOrientationMeasurement;
using flexkalman::AbsolutePositionMeasurement;

static void dumpState(State const &state, const char msg[], size_t iteration) {
    std::cout << "\n"
              << msg << " (iteration " << iteration << "):\n"
              << state.stateVector().transpose() << std::endl;
}

template <typename PM, typename M>
static void runFilterAndCheck(State &state,
                              flexkalman::ProcessModelBase<PM> &processModel,
                              flexkalman::MeasurementBase<M> &meas, double dt,
                              size_t iteration) {
    INFO("Iteration " << iteration);
    INFO("prediction step");
    flexkalman::predict(state, processModel, dt);
    dumpState(state, "After prediction", iteration);
    REQUIRE_FALSE(stateContentsInvalid(state));
    REQUIRE_FALSE(covarianceContentsInvalid(state));

    REQUIRE_FALSE(
        covarianceContentsInvalid(meas.derived().getCovariance(state)));

    INFO("correction step");
    flexkalman::correctUnscented(state, meas.derived());
    dumpState(state, "After correction", iteration);
    REQUIRE_FALSE(stateContentsInvalid(state));
    REQUIRE_FALSE(covarianceContentsInvalid(state));
}

TEMPLATE_TEST_CASE("ProcessModelStability", "",
                   flexkalman::pose_exp_map::ConstantVelocityProcessModel) {

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

    SECTION("AbsoluteOrientationMeasurementConstantAngVel") {
        auto angleAxis = Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitX());
        for (iteration = 0; iteration < 100; ++iteration) {
            angleAxis.angle() += 0.001;
            auto meas = AbsoluteOrientationMeasurement{
                Eigen::Quaterniond(angleAxis),
                Eigen::Vector3d(0.00001, 0.00001, 0.00001)};
            runFilterAndCheck(state, processModel, meas, 0.1, iteration);
        }
        // Can't use isApprox to compare to zero vector
        CHECK(state.position().isMuchSmallerThan(0.001));

        INFO("Should almost reach the most recent measurement");
        CHECK(state.rotationVector()[0] ==
              Approx(angleAxis.angle()).epsilon(0.1));
        CHECK(state.rotationVector()[1] == Approx(0).margin(0.01));
        CHECK(state.rotationVector()[2] == Approx(0).margin(0.01));
        INFO("We shouldn't overshoot")
        CHECK(state.rotationVector().norm() < angleAxis.angle());
    }
}
