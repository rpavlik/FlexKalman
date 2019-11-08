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

#include "FlexKalman/EigenQuatExponentialMap.h"

#include "TestHelpers.h"
#include <catch2/catch.hpp>

TEST_CASE("small_angle_quat_exp") {
    // starts to fail at (0.05, 0, 0) - so that's the limit of "small angle" for
    // now.
    auto rx100 = GENERATE(range(0, 100));
    SECTION("Comparison") {
        auto rotVec = Eigen::Vector3d{rx100 / 2000., 0, 0};
        CAPTURE(rotVec.transpose());
        auto full_exp = flexkalman::util::quat_exp(rotVec);
        CAPTURE(full_exp);
        auto small_angle = flexkalman::util::small_angle_quat_exp(rotVec);
        CAPTURE(small_angle);

        auto approx = Approx::custom();
        approx.margin(0.0001);
        REQUIRE(full_exp.w() == approx(small_angle.w()));
        REQUIRE(full_exp.x() == approx(small_angle.x()));
        REQUIRE(full_exp.y() == approx(small_angle.y()));
        REQUIRE(full_exp.z() == approx(small_angle.z()));
    }
}
