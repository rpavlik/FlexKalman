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
#include "UsefulQuaternions.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
#include <type_traits>

namespace videotracker {
namespace uvbi {
#if 0
    inline void pinholeCameraFlipPose(Eigen::Ref<Eigen::Vector3d> xlate,
                                      Eigen::Quaterniond &rot) {
#endif
    template <typename VecType>
    inline void pinholeCameraFlipPose(VecType &&xlate,
                                      Eigen::Quaterniond &rot) {
#if 0
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        static_assert(std::is_same<double, typename Derived::Scalar>::value,
                      "Translation scalar type must also be double.");
#endif

        /// invert position
        xlate *= -1;
        /// Rotate orientation 180 about Z.
        rot = get180aboutZ() * rot;
    }

#if 0
    inline void
    pinholeCameraFlipVelocities(Eigen::Ref<Eigen::Vector3d> &linVel,
                                Eigen::Ref<Eigen::Vector3d> &angVel) {
#endif
    template <typename Vec1, typename Vec2>
    inline void pinholeCameraFlipVelocities(Vec1 &&linVel, Vec2 &&angVel) {
#if 0
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 3);
        static_assert(std::is_same<typename Derived1::Scalar,
                                   typename Derived2::Scalar>::value,
                      "Velocities must have the same scalar type.");
#endif

        /// invert velocity
        linVel *= -1;
        /// Rotate angular velocity 180 about Z.
        angVel = get180aboutZ() * angVel;
    }

} // namespace uvbi
} // namespace videotracker
