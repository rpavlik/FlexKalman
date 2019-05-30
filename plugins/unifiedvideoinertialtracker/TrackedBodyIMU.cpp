/** @file
    @brief Implementation

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

// Internal Includes
#include "TrackedBodyIMU.h"
#include "AngVelTools.h"
#include "TrackedBody.h"
#include "TrackingSystem.h"
#include <KalmanFramework/EigenExtras.h>

// Library/third-party includes
#include <boost/assert.hpp>

// Standard includes
#include <cmath>

namespace osvr {
namespace vbtracker {
    TrackedBodyIMU::TrackedBodyIMU(TrackedBody &body,
                                   double orientationVariance,
                                   double angularVelocityVariance)
        : m_body(body), m_yaw(0),
          m_useOrientation(getParams().imu.useOrientation),
          m_orientationVariance(orientationVariance),
          m_useAngularVelocity(getParams().imu.useAngularVelocity),
          m_angularVelocityVariance(angularVelocityVariance) {}
    void
    TrackedBodyIMU::updatePoseFromOrientation(util::time::TimeValue const &tv,
                                              Eigen::Quaterniond const &quat) {
        // Choose the equivalent quaternion to the input that makes the data
        // smooth with previous quaternions.
        Eigen::Quaterniond rawSmoothQuat = quat;
        if (m_hasRawQuat) {
            util::flipQuatSignToMatch(rawSmoothQuat, m_rawQuat);
        } else {
            // first report: we'll arbitrarily choose w to be positive.
            if (quat.w() < 0) {
                rawSmoothQuat = Eigen::Quaterniond(-quat.coeffs());
            }
            m_hasRawQuat = true;
        }
        m_rawQuat = rawSmoothQuat;

        if (!m_yawKnown) {
            // This needs to go to calibration instead of to our own pose.
            getBody().getSystem().calibrationHandleIMUData(getBody().getId(),
                                                           tv, rawSmoothQuat);
            return;
        }
        // Save some local state: we do have orientation ourselves now.
        m_quat = transformRawIMUOrientation(rawSmoothQuat);
        m_hasOrientation = true;
        m_last = tv;

        if (!getParams().imu.useOrientation) {
            return;
        }
        // Can it and update the pose with it.
        updatePoseFromMeasurement(tv, preprocessOrientation(tv, rawSmoothQuat));
    }
    void TrackedBodyIMU::updatePoseFromAngularVelocity(
        util::time::TimeValue const &tv, Eigen::Quaterniond const &deltaquat,
        double dt) {
        if (!m_yawKnown) {
            // No calibration yet, and angular velocity isn't useful there.
            return;
        }

        if (!m_useAngularVelocity) {
            return;
        }
        // Can it and update the pose with it.
        updatePoseFromMeasurement(tv,
                                  preprocessAngularVelocity(tv, deltaquat, dt));
    }

    Eigen::Quaterniond TrackedBodyIMU::transformRawIMUOrientation(
        Eigen::Quaterniond const &input) const {
        BOOST_ASSERT_MSG(
            calibrationYawKnown(),
            "transform called before calibration transform known!");
        return m_yawCorrection * input;
        // return input;
    }

    Eigen::Quaterniond TrackedBodyIMU::transformRawIMUAngularVelocity(
        Eigen::Quaterniond const &deltaquat) const {
        BOOST_ASSERT_MSG(
            calibrationYawKnown(),
            "transform called before calibration transform known!");
        /// @todo handle transform for off-center velocity!

        /// Transform for yaw correction.
        /// @todo are the transforms in the right order?
        /// @todo are transforms for yaw correction even needed here? not clear,
        /// since the deltaquat is already in the body coordinate system...
        // return m_yawCorrection.inverse() * deltaquat * m_yawCorrection;
        return deltaquat;
    }

    CannedIMUMeasurement
    TrackedBodyIMU::preprocessOrientation(util::time::TimeValue const &tv,
                                          Eigen::Quaterniond const &quat) {

        auto ret = CannedIMUMeasurement{};
        ret.setYawCorrection(m_yaw);
        ret.setOrientation(transformRawIMUOrientation(quat),
                           Eigen::Vector3d::Constant(m_orientationVariance));
        return ret;
    }

    /// Processes an angular velocity
    CannedIMUMeasurement TrackedBodyIMU::preprocessAngularVelocity(
        util::time::TimeValue const &tv, Eigen::Quaterniond const &deltaquat,
        double dt) {
        Eigen::Vector3d rot =
            incRotToAngVelVec(transformRawIMUAngularVelocity(deltaquat), dt);
        auto ret = CannedIMUMeasurement{};
        ret.setYawCorrection(m_yaw);
        ret.setAngVel(rot,
                      Eigen::Vector3d::Constant(m_angularVelocityVariance));
        return ret;
    }

    bool TrackedBodyIMU::updatePoseFromMeasurement(
        util::time::TimeValue const &tv, CannedIMUMeasurement const &meas) {
        if (!meas.orientationValid() && !meas.angVelValid()) {
            return false;
        }
        getBody().incorporateNewMeasurementFromIMU(tv, meas);
        return true;
    }

    ConfigParams const &TrackedBodyIMU::getParams() const {
        return getBody().getParams();
    }
} // namespace vbtracker
} // namespace osvr
