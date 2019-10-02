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
#include "PoseEstimator_RANSACKalman.h"

// Library/third-party includes
#include "FlexKalman/AbsoluteOrientationMeasurement.h"
#include "FlexKalman/AbsolutePositionMeasurement.h"
#include "FlexKalman/FlexibleKalmanFilter.h"

// Standard includes
// - none

namespace videotracker {
namespace uvbi {
    RANSACKalmanPoseEstimator::RANSACKalmanPoseEstimator(
        double positionVarianceScale, double orientationVariance)
        : m_positionVarianceScale(positionVarianceScale),
          m_orientationVariance(orientationVariance) {}

    bool RANSACKalmanPoseEstimator::
    operator()(EstimatorInOutParams const &p, LedPtrList const &leds,
               videotracker::util::TimeValue const &frameTime) {

        Eigen::Vector3d xlate;
        Eigen::Quaterniond quat;
        /// Call the main pose estimation to get the vector and quat.
        {
            auto ret = m_ransac(p.camParams, leds, p.beacons, p.beaconDebug,
                                xlate, quat);
            if (!ret) {
                return false;
            }
        }

        /// If we got something, filter it in!

        if (p.startingTime != frameTime) {
            /// Predict first if appropriate.
            auto dt = util::time::duration(frameTime, p.startingTime);
            flexkalman::predict(p.state, p.processModel, dt);
        }

        /// Filter in the orientation
        {
            flexkalman::AbsoluteOrientationEKFMeasurement<BodyState> meas(
                quat, Eigen::Vector3d::Constant(m_orientationVariance));
            flexkalman::correct(p.state, p.processModel, meas);
        }
        /// Filter in the orientation
        {
            /// we'll say variance goes up with distance squared.
            flexkalman::AbsolutePositionEKFMeasurement<BodyState> meas(
                xlate, Eigen::Vector3d::Constant(m_positionVarianceScale *
                                                 xlate.z() * xlate.z()));
            flexkalman::correct(p.state, p.processModel, meas);
        }
        return true;
    }
} // namespace uvbi
} // namespace videotracker
