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
#include "CannedIMUMeasurement.h"
#include "ModelTypes.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <KalmanFramework/Angles.h>
#include <KalmanFramework/TimeValue.h>

// Standard includes
// - none

namespace osvr {
namespace vbtracker {
    using flexkalman::util::Angle;
    struct ConfigParams;
    class TrackedBody;
    class TrackedBodyIMU {
      public:
        TrackedBodyIMU(TrackedBody &body, double orientationVariance,
                       double angularVelocityVariance);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TrackedBody &getBody() { return m_body; }
        TrackedBody const &getBody() const { return m_body; }

        /// Processes an orientation
        void updatePoseFromOrientation(util::time::TimeValue const &tv,
                                       Eigen::Quaterniond const &quat);

        /// Processes an angular velocity
        void updatePoseFromAngularVelocity(util::time::TimeValue const &tv,
                                           Eigen::Quaterniond const &deltaquat,
                                           double dt);

        bool hasPoseEstimate() const { return m_hasOrientation; }
        util::time::TimeValue const &getLastUpdate() const { return m_last; }
        /// This estimate incorporates the calibration yaw correction.
        Eigen::Quaterniond const &getPoseEstimate() const { return m_quat; }

        bool calibrationYawKnown() const { return m_yawKnown; }
        void setCalibrationYaw(Angle yaw) {
            using namespace Eigen;
            using namespace util;
            m_yaw = yaw;
            m_yawCorrection =
                Quaterniond(AngleAxisd(getRadians(m_yaw), Vector3d::UnitY()));
            m_yawKnown = true;
        }

        /// Gets the transform based on the calibration yaw
        Eigen::Quaterniond const &getIMUToRoom() const {
            return m_yawCorrection;
        }

      private:
        /// Apply the yaw transform required for "cameraIsForward"
        Eigen::Quaterniond
        transformRawIMUOrientation(Eigen::Quaterniond const &input) const;
        /// Apply the yaw transform required for "cameraIsForward"
        Eigen::Quaterniond transformRawIMUAngularVelocity(
            Eigen::Quaterniond const &deltaquat) const;

        /// Takes in raw delta quats, dt, and timestamps, transforms them, and
        /// spits out a "canned" measurement that can be stored and incorporated
        /// into state.
        CannedIMUMeasurement
        preprocessAngularVelocity(util::time::TimeValue const &tv,
                                  Eigen::Quaterniond const &deltaquat,
                                  double dt);

        /// Takes in raw (untransformed) quats and timestamps, transforms them,
        /// and spits out a "canned" measurement that can be stored and
        /// incorporated into state.
        CannedIMUMeasurement
        preprocessOrientation(util::time::TimeValue const &tv,
                              Eigen::Quaterniond const &quat);

        /// Takes in timestamps and a canned measurement and passes it to the
        /// body to incorporate into state.
        /// @return false if you pass a completely invalid/empty canned
        /// measurement.
        bool updatePoseFromMeasurement(util::time::TimeValue const &tv,
                                       CannedIMUMeasurement const &meas);

        ConfigParams const &getParams() const;
        TrackedBody &m_body;
        bool m_yawKnown = false;
        Angle m_yaw;
        /// this is rRi
        Eigen::Quaterniond m_yawCorrection;

        bool m_useOrientation;
        double m_orientationVariance;
        bool m_useAngularVelocity;
        double m_angularVelocityVariance;

        bool m_hasRawQuat = false;
        /// without yaw correction
        Eigen::Quaterniond m_rawQuat;

        bool m_hasOrientation = false;
        /// measurement in room space (corrected for yaw)
        Eigen::Quaterniond m_quat;
        util::time::TimeValue m_last;
    };
} // namespace vbtracker
} // namespace osvr
