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
#include "ConfigParams.h"
#include "ModelTypes.h"
#include "PoseEstimatorTypes.h"
#include "TrackedBodyTarget.h"

// Library/third-party includes
// - none

// Standard includes
#include <random>

namespace osvr {
namespace vbtracker {

    class SCAATKalmanPoseEstimator {
      public:
        enum class TriBool { False, True, Unknown };
        enum class TrackingHealth {
            Functioning,
            NeedsHardResetNow,
            SoftResetWhenBeaconsSeen
        };
        SCAATKalmanPoseEstimator(ConfigParams const &params);
        bool operator()(EstimatorInOutParams const &p, LedPtrList const &leds,
                        osvr::util::time::TimeValue const &frameTime,
                        double videoDt);

        /// Given a list of LED pointers, filters them out according to
        /// configured parameters, updates the debug data, and returns a list of
        /// just those LEDs to process.
        LedPtrList filterLeds(LedPtrList const &leds, const bool skipBright,
                              const bool skipAll, std::size_t &numBad,
                              EstimatorInOutParams const &p);

        void resetCounters() {
            m_framesInProbation = 0;
            m_framesWithoutIdentifiedBlobs = 0;
            m_framesWithoutUtilizedMeasurements = 0;
            m_lastUsableBeaconsSeen = SIGNAL_HAVE_NOT_SEEN_BEACONS_YET;
            m_ledsUsed = 0;
            m_ledsConsideredMisidentifiedLastFrame = 0;
            m_ledsUsedLastFrame = 0;
            m_misIDConsideredOurFault = false;
        }

        /// Determines whether the Kalman filter is in good working condition,
        /// should fall back to RANSAC immediately, or should fall back next
        /// time beacons are detected. When the algorithm switches back to
        /// Kalman, be sure to call resetCounters()
        TrackingHealth getTrackingHealth();

      private:
        TriBool inBoundingBoxRatioRange(Led const &led);

        void markAsPossiblyMisidentified(Led &led);
        void markAsUsed(Led &led);
        void handlePossiblyMisidentifiedLeds();
        double getVarianceFromBeaconDepth(double depth);

        float m_maxBoxRatio;
        float m_minBoxRatio;
        const bool m_shouldSkipBright;
        const double m_maxResidual;
        const double m_maxSquaredResidual;
        const double m_maxZComponent;
        const double m_highResidualVariancePenalty;
        const double m_beaconProcessNoise;
        const double m_noveltyPenaltyBase;
        const double m_noBeaconLinearVelocityDecayCoefficient;
        const double m_measurementVarianceScaleFactor;
        const double m_brightLedVariancePenalty;
        const double m_distanceMeasVarianceBase;
        const double m_distanceMeasVarianceIntercept;
        const bool m_extraVerbose;
        std::mt19937 m_randEngine;
        static const int SIGNAL_HAVE_NOT_SEEN_BEACONS_YET = -1;
        int m_lastUsableBeaconsSeen = SIGNAL_HAVE_NOT_SEEN_BEACONS_YET;
        std::size_t m_framesInProbation = 0;
        std::size_t m_framesWithoutIdentifiedBlobs = 0;
        std::size_t m_framesWithoutUtilizedMeasurements = 0;
        std::vector<Led *> m_possiblyMisidentified;
        std::size_t m_ledsUsed = 0;
        std::size_t m_ledsConsideredMisidentifiedLastFrame = 0;
        std::size_t m_ledsUsedLastFrame = 0;
        bool m_misIDConsideredOurFault = false;
    };
} // namespace vbtracker
} // namespace osvr
