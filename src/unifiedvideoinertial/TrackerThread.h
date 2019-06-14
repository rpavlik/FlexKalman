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
#include "ThreadsafeBodyReporting.h"
#include "unifiedvideoinertial/ImageSources/ImageSource.h"
#include "unifiedvideoinertial/TrackingSystem.h"
#include "videotrackershared/CameraParameters.h"

// Library/third-party includes
#include "ClientReportTypesC.h"

#include <opencv2/core/core.hpp> // for basic OpenCV types

#include <folly/ProducerConsumerQueue.h>
#include <folly/sorted_vector_types.h>

// Standard includes
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <iosfwd>
#include <mutex>
#include <queue>
#include <thread>

namespace videotracker {
namespace uvbi {
    static const std::chrono::milliseconds IMU_OVERRIDE_SPACING{2};

    static const auto MAX_DEBUG_BEACONS = 34;
    static const auto DEBUG_ANALOGS_PER_BEACON = 6;
    static const auto DEBUG_ANALOGS_REQUIRED =
        MAX_DEBUG_BEACONS * DEBUG_ANALOGS_PER_BEACON;

    using DebugArray = std::array<double, DEBUG_ANALOGS_REQUIRED>;
    struct BodyIdOrdering {
        bool operator()(BodyId const &lhs, BodyId const &rhs) const {
            return lhs.value() < rhs.value();
        }
    };

    using UpdatedBodyIndices = folly::sorted_vector_set<BodyId, BodyIdOrdering>;

    class ImageProcessingThread;

    class TrackerThread {
      public:
        TrackerThread(TrackingSystem &trackingSystem, ImageSource &imageSource,
                      BodyReportingVector &reportingVec,
                      CameraParameters const &camParams,
                      std::int32_t cameraUsecOffset = 0, bool bufferImu = false,
                      bool debugData = false);
        ~TrackerThread();

        // Non-copyable
        TrackerThread(TrackerThread const &) = delete;
        TrackerThread &operator=(TrackerThread const &) = delete;

        /// Thread function-call operator: should be invoked by a lambda in a
        /// dedicated thread.
        void threadAction();

        /// @name Main thread methods
        /// @{
        /// The thread starts and immediately blocks. Calling this allows it to
        /// proceed with execution.
        void permitStart();

        /// Call from the main thread to trigger this thread's execution to exit
        /// after the current frame.
        void triggerStop();

        /// Submit an orientation report for an IMU
        /// @return false if there is no room in the queue for the message
        bool submitIMUReport(TrackedBodyIMU &imu, util::TimeValue const &tv,
                             OSVR_OrientationReport const &report);
        /// @overload
        /// for angular velocity
        bool submitIMUReport(TrackedBodyIMU &imu, util::TimeValue const &tv,
                             OSVR_AngularVelocityReport const &report);

        bool checkForDebugData(DebugArray &data);
        /// @}

        /// Call from image processing thread to signal completion of frame
        /// processing.
        void signalImageProcessingComplete(ImageOutputDataPtr &&imageData,
                                           cv::Mat const &frame,
                                           cv::Mat const &frameGray);

      private:
        /// Helper providing a prefixed output stream for normal messages.
        std::ostream &msg() const;
        /// Helper providing a prefixed output stream for warning messages.
        std::ostream &warn() const;

        /// Main function called repeatedly, once for each (attempted) frame of
        /// video.
        void doFrame();

        /// Can call as soon as the loop starts (as soon as m_numBodies is
        /// known)
        void setupReportingVectorProcessModels();

        /// Should call only once room calibration is completed.
        bool setupReportingVectorRoomTransforms();

        /// Copy updated body state into the reporting vector.
        void updateReportingVector(UpdatedBodyIndices const &bodyIds);

        /// Copy a single body's updated state into the reporting vector. Does
        /// not handle additional derived reports (like HMD in IMU space, etc.)
        /// - just reports the single body.
        void updateReportingVector(BodyId const bodyId);

        /// This function is responsible for triggering the image capture and
        /// processing asynchronously in a separate thread.
        void launchTimeConsumingImageStep();

        std::pair<BodyId, ImuMessageCategory>
        processIMUMessage(IMUMessage const &m);

        /// pointer to body reporting object for camera pose in "room" space
        BodyReporting *getCamPoseReporting() const;
        /// pointer to body reporting object for IMU
        BodyReporting *getIMUReporting() const;
        /// pointer to body reporting object for IMU in camera space
        BodyReporting *getIMUCamReporting() const;
        /// pointer to body reporting for HMD in camera space
        BodyReporting *getHMDCamReporting() const;

        void updateExtraCameraReport();
        void updateExtraIMUReports();

        TrackingSystem &m_trackingSystem;
        ImageSource &m_cam;
        BodyReportingVector &m_reportingVec;
        CameraParameters m_camParams;
        std::size_t m_numBodies = 0; //< initialized when loop started.
        const std::int32_t m_cameraUsecOffset = 0;

        /// Whether we should wait a period of time before updating the
        /// reporting vector with just IMU reports (compared to updating
        /// reporting vector with all new data)
        const bool m_bufferImu = false;

        const bool m_debugData = false;

        using our_clock = std::chrono::steady_clock;

        bool shouldSendImuReport() {
            auto now = our_clock::now();
            if (now > m_nextImuOverrideReport) {
                m_nextImuOverrideReport = now + IMU_OVERRIDE_SPACING;
                return true;
            }
            return false;
        }

        void setImuOverrideClock() {
            m_nextImuOverrideReport = our_clock::now() + IMU_OVERRIDE_SPACING;
        }

        our_clock::time_point m_nextImuOverrideReport;
        optional<our_clock::time_point> m_nextCameraPoseReport;

        /// a void promise, as suggested by Scott Meyers, to hold the thread
        /// operation at the beginning until we want it to really start running.
        std::promise<void> m_startupSignal;

        bool m_setCameraPose = false;

        /// @name Updated asynchronously by timeConsumingImageStep()
        /// @{
        cv::Mat m_frame;
        cv::Mat m_frameGray;
        ImageOutputDataPtr m_imageData;
        /// @}

        /// @name Run flag
        /// @{
        std::mutex m_runMutex;
        bool m_run = true;
        /// @}

        /// @name Message queue for async image processing and receiving IMU
        /// reports from other threads.
        /// @{
        std::condition_variable m_messageCondVar;
        std::mutex m_messageMutex;
        bool m_timeConsumingImageStepComplete = false;
        folly::ProducerConsumerQueue<IMUMessage> m_imuMessages;
        /// @}

        folly::ProducerConsumerQueue<DebugArray> m_debugDataMessages;

        ImageProcessingThread *imageProcThreadObj_ = nullptr;

        /// The thread used by timeConsumingImageStep()
        std::thread m_imageThread;
    };
} // namespace uvbi
} // namespace videotracker
