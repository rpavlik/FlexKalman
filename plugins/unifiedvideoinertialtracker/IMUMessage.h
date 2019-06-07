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
#include <KalmanFramework/ClientReportTypesC.h>
#include <KalmanFramework/TimeValue.h>
#include <nonstd/variant.hpp>

// Standard includes
// - none

namespace osvr {
namespace vbtracker {
    using nonstd::monostate;
    using nonstd::variant;

    // Forward declaration
    class TrackedBodyIMU;

    enum class ImuMessageCategory { Empty, Orientation, AngularVelocity };

    /// An IMU report data structure, along with the report time
    /// and the internal tracking system's pointer to IMU object.
    template <typename ReportType> class TimestampedImuReport {
      public:
        TimestampedImuReport(TrackedBodyIMU &myImu,
                             util::time::TimeValue const &tv,
                             ReportType const &d)
            : imuPtr(&myImu), timestamp(tv), data(d) {}

      private:
        TrackedBodyIMU *imuPtr;

      public:
        TrackedBodyIMU &imu() const { return *imuPtr; }

        util::time::TimeValue timestamp;
        ReportType data;
    };

    /// Generic constructor/factory function
    template <typename ReportType>
    inline TimestampedImuReport<ReportType>
    makeImuReport(TrackedBodyIMU &myImu, util::time::TimeValue const &tv,
                  ReportType const &d) {
        return TimestampedImuReport<ReportType>{myImu, tv, d};
    }

    /// An orientation report data structure, along with the report time
    /// and the internal tracking system's pointer to IMU object.
    using TimestampedOrientation = TimestampedImuReport<OSVR_OrientationReport>;

    /// An angular velocity report data structure, along with the report time
    /// and the internal tracking system's pointer to IMU object.
    using TimestampedAngVel = TimestampedImuReport<OSVR_AngularVelocityReport>;

    /// A "typesafe tagged-union" variant that can hold an IMU report data along
    /// with the timestamp and the pointer to the tracking system's IMU object
    /// that it applies to.
    using IMUMessage =
        variant<monostate, TimestampedOrientation, TimestampedAngVel>;

} // namespace vbtracker
} // namespace osvr
