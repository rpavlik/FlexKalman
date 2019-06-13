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
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace videotracker {
namespace uvbi {
    inline Eigen::Quaterniond get180aboutX() {
        // all zero except x = 1 produces 180 about x
        return Eigen::Quaterniond(0, 1, 0, 0);
    }
    inline Eigen::Quaterniond get180aboutY() {
        // all zero except y = 1 produces 180 about y
        return Eigen::Quaterniond(0, 0, 1, 0);
    }
    inline Eigen::Quaterniond get180aboutZ() {
        // all zero except z = 1 produces 180 about z
        return Eigen::Quaterniond(0, 0, 0, 1);
    }
} // namespace uvbi
} // namespace videotracker
