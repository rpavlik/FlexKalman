/** @file
    @brief Header

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
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

#pragma once

// Internal Includes
// - none

// Library/third-party includes
#include "FlexKalman/EigenQuatExponentialMap.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
#include <cmath>

namespace videotracker {
namespace uvbi {
    /// use only for derivatives - has factor of 2/0.5 in it!
    inline Eigen::Quaterniond
    angVelVecToIncRot(Eigen::Vector3d const &angVelVec, double dt) {
        return flexkalman::util::quat_exp(angVelVec * dt * 0.5).normalized();
    }

    /// use only for derivatives - has factor of 2/0.5 in it!
    inline Eigen::Vector3d incRotToAngVelVec(Eigen::Quaterniond const &incRot,
                                             double dt) {
        return flexkalman::util::quat_ln(incRot) * 2. / dt;
    }

} // namespace uvbi
} // namespace videotracker
