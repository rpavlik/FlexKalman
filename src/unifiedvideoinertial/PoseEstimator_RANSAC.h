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
#include "PoseEstimatorTypes.h"
#include "unifiedvideoinertial/ConfigParams.h"

// Library/third-party includes
// - none

// Standard includes
#include <cstddef>

namespace videotracker {
namespace uvbi {
    class RANSACPoseEstimator {
      public:
        /// Perform RANSAC-based pose estimation.
        ///
        /// @param[out] outXlate translation output parameter
        /// @param[out] outQuat rotation output parameter
        /// @param skipBrightsCutoff If positive, the number of non-bright LEDs
        /// seen that will trigger us to skip using bright LEDs in pose
        /// estimation.
        /// @return true if a pose was estimated.
        bool operator()(CameraParameters const &camParams,
                        LedPtrList const &leds, BeaconStateVec const &beacons,
                        std::vector<BeaconData> &beaconDebug,
                        Eigen::Vector3d &outXlate, Eigen::Quaterniond &outQuat,
                        int skipBrightsCutoff = -1, std::size_t iterations = 5);

        /// Perform RANSAC-based pose estimation and use it to update a body
        /// state (state vector and error covariance)
        ///
        /// @param[out] state Tracked body state that will be updated if a pose
        /// was estimated
        /// @return true if a pose was estimated.
        bool operator()(EstimatorInOutParams const &p, LedPtrList const &leds);

      private:
        const std::size_t m_requiredInliers = 4;
        const std::size_t m_permittedOutliers = 0;
    };
} // namespace uvbi
} // namespace videotracker
