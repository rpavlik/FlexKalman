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
#include "IMUMessage.h"
#include "TrackedBodyIMU.h"
#include "unifiedvideoinertial/TrackedBody.h"

// Library/third-party includes
#include "EigenInterop.h"
#include "unifiedvideoinertial/nonstd/variant.hpp"

// Standard includes
// - none

namespace videotracker {
namespace uvbi {
    namespace detail {
        using nonstd::visit;

        /// Implementation detail of unpacking and handling the IMU messages.
        class IMUMessageProcessor {
          public:
            IMUMessageProcessor(BodyId &id, ImuMessageCategory &msgType)
                : bodyId(id), messageType(msgType) {}
            BodyId &bodyId;
            ImuMessageCategory &messageType;

            template <typename Report>
            void setBodyIdFromReport(Report const &report) const {
                bodyId = report.imu().getBody().getId();
            }

            void operator()(monostate const &) const {
                /// dummy overload to handle empty messages
            }

            void operator()(TimestampedImuReport<OSVR_OrientationReport> const
                                &report) const {
                messageType = ImuMessageCategory::Orientation;
                setBodyIdFromReport(report);
                report.imu().updatePoseFromOrientation(
                    report.timestamp,
                    util::eigen_interop::map(report.data.rotation).quat());
            }

            void operator()(
                TimestampedImuReport<OSVR_AngularVelocityReport> const &report)
                const {
                setBodyIdFromReport(report);
                messageType = ImuMessageCategory::AngularVelocity;
                report.imu().updatePoseFromAngularVelocity(
                    report.timestamp,
                    util::eigen_interop::map(
                        report.data.state.incrementalRotation)
                        .quat(),
                    report.data.state.dt);
            }
        };
    } // namespace detail
    inline std::pair<BodyId, ImuMessageCategory>
    processImuMessage(IMUMessage const &m) {

        BodyId bodyId;
        ImuMessageCategory messageType = ImuMessageCategory::Empty;
        detail::IMUMessageProcessor processor{bodyId, messageType};
        visit(processor, m);
        return std::make_pair(bodyId, messageType);
    }
} // namespace uvbi
} // namespace videotracker
