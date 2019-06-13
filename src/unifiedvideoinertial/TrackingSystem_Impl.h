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
#include "RoomCalibration.h"
#include "unifiedvideoinertial/ConfigParams.h"
#include "unifiedvideoinertial/TrackingSystem.h"
#include "videotrackershared/CameraParameters.h"
#include "videotrackershared/GenericBlobExtractor.h"

// Library/third-party includes
#include "unifiedvideoinertial/TimeValue.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

// Standard includes
#include <memory>

namespace videotracker {
namespace uvbi {
    class TrackingDebugDisplay;

    /// Private implementation structure for TrackingSystem
    struct TrackingSystem::Impl {
        Impl(ConfigParams const &params);
        ~Impl();

        // noncopyable
        Impl(Impl const &) = delete;
        Impl &operator=(Impl const &) = delete;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void triggerDebugDisplay(TrackingSystem &tracking);

        /// @name Cached data from the ImageProcessingOutput updated in phase 2
        /// @{
        /// Cached copy of the last grey frame
        cv::Mat frame;
        /// Cached copy of the last grey frame
        cv::Mat frameGray;
        /// Cached copy of the last (undistorted) camera parameters to be used.
        CameraParameters camParams;
        util::TimeValue lastFrame;
        /// @}
        bool roomCalibCompleteCached = false;

        bool haveCameraPose = false;
        Eigen::Isometry3d cameraPose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d cameraPoseInv = Eigen::Isometry3d::Identity();

        RoomCalibration calib;

        LedUpdateCount updateCount;
        BlobExtractorPtr blobExtractor;
        std::unique_ptr<TrackingDebugDisplay> debugDisplay;
    };

} // namespace uvbi
} // namespace videotracker
