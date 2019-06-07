/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
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
#include <KalmanFramework/TimeValue.h>
#include <opencv2/core/core.hpp>

// Standard includes
#include <memory>

namespace osvr {
namespace vbtracker {
    /// Uniform interface for the various normal to strange image sources for
    /// the tracking algorithm.
    class ImageSource {
      public:
        /// Destructor
        virtual ~ImageSource();

        /// @return true if the camera/image source is OK
        virtual bool ok() const = 0;

        /// Trigger camera capture. May not necessarily include retrieval.
        /// Blocks until an image is available. or failure occurs.
        ///
        /// Timestamp after this call returns.
        ///
        /// @return false if the camera failed.
        virtual bool grab() = 0;

        /// Call after grab() to get the actual image data.
        virtual void retrieve(cv::Mat &color, cv::Mat &gray,
                              osvr::util::time::TimeValue &timestamp);

        /// @overload
        /// discards timestamp.
        inline void retrieve(cv::Mat &color, cv::Mat &gray) {
            osvr::util::time::TimeValue ts;
            retrieve(color, gray, ts);
        }

        /// Get resolution of the images from this source.
        virtual cv::Size resolution() const = 0;

        /// For those devices that naturally read a non-corrupt color image,
        /// overriding just this method will let the default implementation of
        /// retrieve() do the RGB to Gray for you.
        virtual void retrieveColor(cv::Mat &color,
                                   osvr::util::time::TimeValue &timestamp) = 0;
        /// @overload
        /// discards timestamp.
        inline void retrieveColor(cv::Mat &color) {
            osvr::util::time::TimeValue ts;
            retrieveColor(color, ts);
        }

      protected:
        ImageSource() = default;
    };

    using ImageSourcePtr = std::unique_ptr<ImageSource>;
} // namespace vbtracker
} // namespace osvr
