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
#include "videotrackershared/Assert.h"

// Library/third-party includes
#include "Angles.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
#include <array>

namespace videotracker {
namespace uvbi {

    using util::Angle;
    /// A safe way to store and transport an orientation measurement or an
    /// angular velocity measurement without needing special alignment
    class CannedIMUMeasurement {
      public:
        void setYawCorrection(Angle y) { m_yawCorrection = y; }
        Angle getYawCorrection() const { return m_yawCorrection; }
        void setOrientation(Eigen::Quaterniond const &quat,
                            Eigen::Vector3d const &variance) {
            Eigen::Vector4d::Map(m_quat.data()) = quat.coeffs();
            Eigen::Vector3d::Map(m_quatVar.data()) = variance;
            m_orientationValid = true;
        }
        bool orientationValid() const { return m_orientationValid; }

        void restoreQuat(Eigen::Quaterniond &quat) const {
            VIDEOTRACKER_ASSERT_MSG(
                orientationValid(),
                "restoring quat on an invalid orientation measurement!");
            quat.coeffs() = Eigen::Vector4d::Map(m_quat.data());
        }

        void restoreQuatVariance(Eigen::Vector3d &var) const {
            VIDEOTRACKER_ASSERT_MSG(orientationValid(),
                                    "restoring quat variance on "
                                    "an invalid orientation "
                                    "measurement!");
            var = Eigen::Vector3d::Map(m_quatVar.data());
        }

        void setAngVel(Eigen::Vector3d const &angVel,
                       Eigen::Vector3d const &variance) {
            Eigen::Vector3d::Map(m_angVel.data()) = angVel;
            Eigen::Vector3d::Map(m_angVelVar.data()) = variance;
            m_angVelValid = true;
        }

        bool angVelValid() const { return m_angVelValid; }
        void restoreAngVel(Eigen::Vector3d &angVel) const {
            VIDEOTRACKER_ASSERT_MSG(angVelValid(), "restoring ang vel on "
                                                   "an invalid ang vel "
                                                   "measurement!");
            angVel = Eigen::Vector3d::Map(m_angVel.data());
        }
        void restoreAngVelVariance(Eigen::Vector3d &var) const {
            VIDEOTRACKER_ASSERT_MSG(angVelValid(),
                                    "restoring ang vel variance on "
                                    "an invalid ang vel "
                                    "measurement!");
            var = Eigen::Vector3d::Map(m_angVelVar.data());
        }

      private:
        Angle m_yawCorrection;
        bool m_orientationValid = false;
        std::array<double, 4> m_quat;
        std::array<double, 3> m_quatVar;
        bool m_angVelValid = false;
        std::array<double, 3> m_angVel;
        std::array<double, 3> m_angVelVar;
    };
} // namespace uvbi
} // namespace videotracker
