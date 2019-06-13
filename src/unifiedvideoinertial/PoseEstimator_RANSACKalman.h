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
#include "PoseEstimator_RANSAC.h"
#include "unifiedvideoinertial/ConfigParams.h"
#include "unifiedvideoinertial/Types.h"

// Library/third-party includes
// - none

// Standard includes
#include <cstddef>

namespace videotracker {
namespace uvbi {
    class RANSACKalmanPoseEstimator {
      public:
        RANSACKalmanPoseEstimator(double positionVarianceScale = 1.e-1,
                                  double orientationVariance = 1.e0);
        /// Perform RANSAC-based pose estimation but filter results in via an
        /// EKF to the body state.
        ///
        /// @return true if a pose was estimated.
        bool operator()(EstimatorInOutParams const &p, LedPtrList const &leds,
                        videotracker::util::TimeValue const &frameTime);

      private:
        RANSACPoseEstimator m_ransac;
        const double m_positionVarianceScale;
        const double m_orientationVariance;
    };
} // namespace uvbi
} // namespace videotracker
