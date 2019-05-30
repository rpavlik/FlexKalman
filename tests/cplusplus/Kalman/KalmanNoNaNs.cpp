/** @file
    @brief Implementation

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
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

#define CATCH_CONFIG_MAIN

#include <KalmanFramework/EigenCoreGeometry.h>
#include <iostream>
template <typename T>
inline void dumpKalmanDebugOuput(const char name[], const char expr[],
                                 T const &value) {
    std::cout << "\n(Kalman Debug Output) " << name << " [" << expr << "]:\n"
              << value << std::endl;
}

#define KALMANFRAMEWORK_KALMAN_DEBUG_OUTPUT(Name, Value)                       \
    dumpKalmanDebugOuput(Name, #Value, Value)

// Internal Includes
#include "ContentsInvalid.h"
#include <KalmanFramework/AbsoluteOrientationMeasurement.h>
#include <KalmanFramework/AbsolutePositionMeasurement.h>
#include <KalmanFramework/FlexibleKalmanFilter.h>
#include <KalmanFramework/PoseConstantVelocity.h>
#include <KalmanFramework/PoseDampedConstantVelocity.h>

// Library/third-party includes
#include <catch2/catch.hpp>

// Standard includes
#include <iostream>

using ProcessModel = osvr::kalman::PoseConstantVelocityProcessModel;
using State = ProcessModel::State;
using AbsoluteOrientationMeasurement =
    osvr::kalman::AbsoluteOrientationMeasurement<State>;
using AbsolutePositionMeasurement =
    osvr::kalman::AbsolutePositionMeasurement<State>;
using Filter = osvr::kalman::FlexibleKalmanFilter<ProcessModel>;

class Stability {
  public:
    template <typename Filter> void dumpInitialState(Filter const &filter) {
        REQUIRE(iteration == 0);
        dumpState(filter.state(), "Initial state");
    }

    template <typename State>
    void dumpState(State const &state, const char msg[]) {
        std::cout << "\n"
                  << msg << " (iteration " << iteration << "):\n"
                  << state << std::endl;
    }
    template <typename Filter, typename Measurement>
    void filterAndCheck(Filter &filter, Measurement &meas, double dt = 0.1) {
        INFO("Iteration " << iteration);
        INFO("prediction step");
        filter.predict(dt);
        dumpState(filter.state(), "After prediction");
        REQUIRE_FALSE(stateContentsInvalid(filter.state()));
        REQUIRE_FALSE(covarianceContentsInvalid(filter.state()));

        REQUIRE_FALSE(
            covarianceContentsInvalid(meas.getCovariance(filter.state())));

        INFO("correction step");
        filter.correct(meas);
        dumpState(filter.state(), "After correction");
        REQUIRE_FALSE(stateContentsInvalid(filter.state()));
        REQUIRE_FALSE(covarianceContentsInvalid(filter.state()));
        iteration++;
    }

    template <typename Filter, typename Measurement>
    void filterAndCheckRepeatedly(Filter &filter, Measurement &meas,
                                  double dt = 0.1,
                                  std::size_t iterations = 100) {
        for (iteration = 0; iteration < iterations; iteration++) {
            INFO("Iteration " << iteration);
            INFO("prediction step");
            filter.predict(dt);
            dumpState(filter.state(), "After prediction");
            REQUIRE_FALSE(stateContentsInvalid(filter.state()));
            REQUIRE_FALSE(covarianceContentsInvalid(filter.state()));

            REQUIRE_FALSE(
                covarianceContentsInvalid(meas.getCovariance(filter.state())));

            INFO("correction step");
            filter.correct(meas);
            dumpState(filter.state(), "After correction");
            REQUIRE_FALSE(stateContentsInvalid(filter.state()));
            REQUIRE_FALSE(covarianceContentsInvalid(filter.state()));
        }
    }

  private:
    std::size_t iteration = 0;
};

template <typename T> class VariedProcessModelStability : public Stability {};

TEMPLATE_TEST_CASE(
    "ProcessModelStability_IdentityAbsoluteOrientationMeasurement", "",
    osvr::kalman::PoseConstantVelocityProcessModel,
    osvr::kalman::PoseDampedConstantVelocityProcessModel) {
    VariedProcessModelStability<TestType> fixture;
    using Filter = osvr::kalman::FlexibleKalmanFilter<TestType>;

    auto filter = Filter{};
    auto meas = AbsoluteOrientationMeasurement{
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d(0.00001, 0.00001, 0.00001)};
    fixture.dumpInitialState(filter);
    fixture.filterAndCheckRepeatedly(filter, meas);
    /// @todo check that it's roughly identity
}

TEMPLATE_TEST_CASE("ProcessModelStability_IdentityAbsolutePositionMeasurement",
                   "", osvr::kalman::PoseConstantVelocityProcessModel,
                   osvr::kalman::PoseDampedConstantVelocityProcessModel) {
    VariedProcessModelStability<TestType> fixture;
    using Filter = osvr::kalman::FlexibleKalmanFilter<TestType>;

    auto filter = Filter{};
    auto meas = AbsolutePositionMeasurement{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.000007)};
    fixture.dumpInitialState(filter);
    fixture.filterAndCheckRepeatedly(filter, meas);
    /// @todo check that it's roughly identity
}

TEMPLATE_TEST_CASE("ProcessModelStability_AbsolutePositionMeasurementXlate111",
                   "", osvr::kalman::PoseConstantVelocityProcessModel,
                   osvr::kalman::PoseDampedConstantVelocityProcessModel) {
    VariedProcessModelStability<TestType> fixture;
    using Filter = osvr::kalman::FlexibleKalmanFilter<TestType>;

    auto filter = Filter{};
    auto meas = AbsolutePositionMeasurement{
        Eigen::Vector3d::Constant(1), Eigen::Vector3d::Constant(0.000007)};
    fixture.dumpInitialState(filter);
    fixture.filterAndCheckRepeatedly(filter, meas);
    /// @todo check that it's roughly identity orientation, position of 1, 1, 1
}
