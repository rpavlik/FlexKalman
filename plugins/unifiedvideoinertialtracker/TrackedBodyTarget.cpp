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
#include "TrackedBodyTarget.h"
#include "AssignMeasurementsToLeds.h"
#include "BodyTargetInterface.h"
#include "HDKLedIdentifier.h"
#include "LED.h"
#include "PoseEstimatorTypes.h"
#include "PoseEstimator_RANSAC.h"
#include "PoseEstimator_RANSACKalman.h"
#include "PoseEstimator_SCAATKalman.h"
#include "TrackedBody.h"
#include "cvToEigen.h"
#include <KalmanFramework/CSV.h>
#include <KalmanFramework/CSVCellGroup.h>

// Library/third-party includes
#include <boost/assert.hpp>
#include <util/Stride.h>

// Standard includes
#include <algorithm>
#include <fstream>
#include <iostream>

#undef UVBI_DEBUG_ERROR_VARIANCE_WHEN_TRACKING_LOST
#undef UVBI_DEBUG_ERROR_VARIANCE

#undef UVBI_FRAMEDROP_HEURISTIC_WARNING

#define UVBI_VERBOSE_ERROR_BOUNDS

/// make blobs.csv for determining variance.
#undef UVBI_DUMP_BLOB_CSV

namespace osvr {
namespace vbtracker {
    enum class TargetTrackingState {
        RANSAC,
        RANSACKalman,
        EnteringKalman,
        Kalman,
        RANSACWhenBlobDetected,
        RANSACKalmanWhenBlobDetected
    };

    enum class TargetHealthState {
        OK,
        StopTrackingErrorBoundsExceeded,
        StopTrackingLostSight,
        HardResetNonFiniteState
    };

    static const auto MAX_FRAMES_WITHOUT_BEACONS = 150;

    inline double getLimitOnMaxPositionalErrorVariance(double distance) {
        /// An exponential function was fit to recorded data of max positional
        /// error variances and distances during normal tracking operation,
        /// using libreoffice calc. R^2 = 0.95, and on a plot with a log-scale
        /// for y, the data looked fairly linear.

        /// log-plot slope B is used directly from that regression.
        /// coefficient A hand-picked based on original fit, to move this
        /// function above the normal data with a good margin of error.
        static const double A = 6.4116155459;
        static const double B = 8e-5;
        return B * std::exp(A * distance);
    }

    inline double getMaxPositionalErrorVariance(BodyState const &bodyState) {
        return bodyState.errorCovariance().diagonal().head<3>().maxCoeff();
    }

    /// Predicate to check simply if the state corresponds to running the SCAAT
    /// Kalman filter mode, centralized and named.
    inline bool isStateSCAAT(TargetTrackingState trackingState) {
        return !(trackingState == TargetTrackingState::RANSAC ||
                 trackingState == TargetTrackingState::RANSACKalman);
    }

    class TargetHealthEvaluator {
      public:
        TargetHealthState operator()(BodyState const &bodyState,
                                     LedPtrList const &leds,
                                     TargetTrackingState trackingState) {
            if (!bodyState.stateVector().array().allFinite()) {
                return TargetHealthState::HardResetNonFiniteState;
            }
            if (!bodyState.getQuaternion().coeffs().array().allFinite()) {
                return TargetHealthState::HardResetNonFiniteState;
            }
            if (leds.empty()) {
                m_framesWithoutValidBeacons++;
            } else {
                m_framesWithoutValidBeacons = 0;
            }

            if (isStateSCAAT(trackingState) &&
                m_framesWithoutValidBeacons != 0) {
                auto maxPositionalError =
                    getMaxPositionalErrorVariance(bodyState);
                auto distance = bodyState.position().z();
                auto errorLimit =
                    getLimitOnMaxPositionalErrorVariance(distance);
                if (maxPositionalError > errorLimit) {
                    return TargetHealthState::StopTrackingErrorBoundsExceeded;
                }
            }

            if (m_framesWithoutValidBeacons > MAX_FRAMES_WITHOUT_BEACONS) {
                return TargetHealthState::StopTrackingLostSight;
            }
            return TargetHealthState::OK;
        }

      private:
        std::size_t m_framesWithoutValidBeacons = 0;
    };

    struct TrackedBodyTarget::Impl {
        Impl(ConfigParams const &params, BodyTargetInterface const &bodyIface)
            : bodyInterface(bodyIface), kalmanEstimator(params),
              ransacKalmanEstimator(params.softResetPositionVarianceScale,
                                    params.softResetOrientationVariance),
              permitKalman(params.permitKalman), softResets(params.softResets)

#ifdef UVBI_DUMP_BLOB_CSV
              ,
              blobFile("blobs.csv"), csv(blobFile)
#endif // UVBI_DUMP_BLOB_CSV
        {
        }
        BodyTargetInterface bodyInterface;
        LedGroup leds;
        LedPtrList usableLeds;
        LedIdentifierPtr identifier;
        RANSACPoseEstimator ransacEstimator;
        SCAATKalmanPoseEstimator kalmanEstimator;
        RANSACKalmanPoseEstimator ransacKalmanEstimator;

        TargetHealthEvaluator healthEval;

        TargetTrackingState trackingState = TargetTrackingState::RANSAC;
        TargetTrackingState lastFrameAlgorithm = TargetTrackingState::RANSAC;

        /// Permit as a purely policy measure
        bool permitKalman = true;

        /// whether to use RANSAC Kalman when we don't need a hard reset of
        /// state.
        const bool softResets = false;

        bool hasPrev = false;
        osvr::util::time::TimeValue lastEstimate;

        /// Number of times we've lost or otherwise had to reset tracking, "soft
        /// resets" included.
        std::size_t trackingResets = 0;
        std::ostringstream outputSink;

#ifdef UVBI_DUMP_BLOB_CSV
        std::ofstream blobFile;
        util::StreamCSV csv;
#endif // UVBI_DUMP_BLOB_CSV
    };

    inline BeaconStateVec createBeaconStateVec(ConfigParams const &params,
                                               TargetSetupData const &setupData,
                                               Eigen::Vector3d &beaconOffset) {
        {
            /// Compute or retrieve the beacon offset.
            if (params.offsetToCentroid) {
                Eigen::Vector3d beaconSum = Eigen::Vector3d::Zero();
                auto bNum = size_t{0};
                for (auto &beacon : setupData.locations) {
                    beaconSum += cvToVector(beacon).cast<double>();
                    bNum++;
                }
                beaconOffset = beaconSum / bNum;
                if (params.debug) {
                    std::cout << "[Tracker Target] Computed beacon centroid: "
                              << beaconOffset.transpose() << std::endl;
                }
            } else {
                beaconOffset = Eigen::Vector3d::Map(params.manualBeaconOffset);
            }
        }
        /// Create the vector we'll return, and then the beacon state
        /// objects.
        using size_type = TargetSetupData::size_type;
        const auto n = setupData.numBeacons();
        BeaconStateVec beacons;
        beacons.reserve(n);
        Eigen::Vector3d location;
        for (size_type i = 0; i < n; ++i) {
            location = cvToVector(setupData.locations[i]).cast<double>() -
                       beaconOffset;
            BeaconStatePtr beacon(new BeaconState(
                location, Eigen::Vector3d::Constant(
                              setupData.initialAutocalibrationErrors[i])
                              .asDiagonal()));
            beacons.emplace_back(std::move(beacon));
        }
        return beacons;
    }

    TrackedBodyTarget::TrackedBodyTarget(TrackedBody &body,
                                         BodyTargetInterface const &bodyIface,
                                         Eigen::Vector3d const &targetToBody,
                                         TargetSetupData const &setupData,
                                         TargetId id)
        : m_body(body), m_id(id), m_targetToBody(targetToBody),
          m_numBeacons(setupData.numBeacons()),
          m_beaconMeasurementVariance(setupData.baseMeasurementVariances),
          m_beaconFixed(setupData.isFixed),
          m_beaconEmissionDirection(setupData.emissionDirections),
          m_impl(new Impl(getParams(), bodyIface)) {

        /// Create the beacon state objects and initialize the beacon offset.
        m_beacons =
            createBeaconStateVec(getParams(), setupData, m_beaconOffset);

        /// Make a copy of those beacons.
        for (auto &beacon : m_beacons) {
            m_origBeacons.emplace_back(new BeaconState(*beacon));
        }

        /// Create the beacon debug data
        m_beaconDebugData.resize(m_beacons.size());

#ifdef UVBI_DUMP_BLOB_CSV
        {
            /// Pre-generate all the known beacon ID columns so they are in
            /// order and so we can start streaming the CSV right away.
            for (std::size_t i = 0; i < m_numBeacons; ++i) {
                auto prefix = std::to_string(i + 1) + ".";
                m_impl->csv.getColumn(prefix + "x");
                m_impl->csv.getColumn(prefix + "y");
                m_impl->csv.getColumn(prefix + "diameter");
                m_impl->csv.getColumn(prefix + "area");
                m_impl->csv.getColumn(prefix + "bright");
            }
            m_impl->csv.startOutput();
        }
#endif // UVBI_DUMP_BLOB_CSV
        {
            /// Create the LED identifier
            std::unique_ptr<OsvrHdkLedIdentifier> identifier(
                new OsvrHdkLedIdentifier(setupData.patterns));
            m_impl->identifier = std::move(identifier);
        }
        m_verifyInvariants();
    }

    TrackedBodyTarget::~TrackedBodyTarget() {}

    BodyTargetId TrackedBodyTarget::getQualifiedId() const {
        return BodyTargetId(getBody().getId(), getId());
    }

    Eigen::Vector3d
    TrackedBodyTarget::getBeaconAutocalibPosition(ZeroBasedBeaconId i) const {
        BOOST_ASSERT(!i.empty());
        BOOST_ASSERT_MSG(i.value() < getNumBeacons(),
                         "Beacon ID must be less than number of beacons.");
        BOOST_ASSERT_MSG(i.value() >= 0,
                         "Beacon ID must not be a sentinel value!");
        return m_beacons.at(i.value())->stateVector() + m_beaconOffset;
    }

    Eigen::Vector3d
    TrackedBodyTarget::getBeaconAutocalibVariance(ZeroBasedBeaconId i) const {
        BOOST_ASSERT(!i.empty());
        BOOST_ASSERT_MSG(i.value() < getNumBeacons(),
                         "Beacon ID must be less than number of beacons.");
        BOOST_ASSERT_MSG(i.value() >= 0,
                         "Beacon ID must not be a sentinel value!");
        return m_beacons.at(i.value())->errorCovariance().diagonal();
    }

    void TrackedBodyTarget::resetBeaconAutocalib() {
        m_beacons.clear();
        for (auto &beacon : m_origBeacons) {
            m_beacons.emplace_back(new BeaconState(*beacon));
        }
    }

    std::size_t TrackedBodyTarget::processLedMeasurements(
        LedMeasurementVec const &undistortedLeds) {
        // std::list<LedMeasurement> measurements{begin(undistortedLeds),
        // end(undistortedLeds)};
        LedMeasurementVec measurements{undistortedLeds};
        const auto prevUsableLedCount = usableLeds().size();
        /// Clear the "usableLeds" that will be populated in a later step, if we
        /// get that far.
        usableLeds().clear();

        if (getParams().streamBeaconDebugInfo) {
            /// Only bother resetting if anyone is actually going to receive the
            /// data.
            for (auto &data : m_beaconDebugData) {
                data.reset();
            }
        }

        const auto blobMoveThreshold = getParams().blobMoveThreshold;
        const auto blobsKeepIdentity = getParams().blobsKeepIdentity;
        auto &myLeds = m_impl->leds;

        const auto prevLedCount = myLeds.size();

        const auto numMeasurements = measurements.size();

        AssignMeasurementsToLeds assignment(myLeds, undistortedLeds,
                                            m_numBeacons, blobMoveThreshold);

        assignment.populateStructures();
        static const auto HEAP_PREFIX = "[ASSIGN HEAP] ";
        bool verbose = false;
        if (getParams().extraVerbose) {
            // if (getParams().debug) {
            static ::util::Stride assignStride(157);
            assignStride++;
            if (assignStride) {
                verbose = true;
            }
        }
        if (verbose) {
            std::cout << HEAP_PREFIX << "Heap contains " << assignment.size()
                      << " elts, of possible "
                      << assignment.theoreticalMaxSize() << " (ratio "
                      << assignment.heapSizeFraction() << ")" << std::endl;
        }
        while (assignment.hasMoreMatches()) {
            auto ledAndMeasurement = assignment.getMatch();
            auto &led = ledAndMeasurement.first;
            auto &meas = ledAndMeasurement.second;
            led.addMeasurement(meas, blobsKeepIdentity);
            if (handleOutOfRangeIds(led, m_numBeacons)) {
                /// For some reason, filtering in that measurement caused an LED
                /// object to go bad. The above function wiped the LED object,
                /// but let's undo the match and usage of the measurement in
                /// case it was someone else's.
                auto success = assignment.resumbitMeasurement(meas);
                std::cerr << "ERROR: We just got a faulty one: filtering in "
                             "measurement from "
                          << meas.loc
                          << " made an LED go invalid. The measurement "
                          << (success ? "could" : "could NOT")
                          << " be resubmitted successfully\n";
            }
        }
        if (verbose) {
            const auto numUnclaimedLedObjects =
                assignment.numUnclaimedLedObjects();
            const auto numUnclaimedMeasurements =
                assignment.numUnclaimedMeasurements();
            const auto usedMeasurements =
                numMeasurements - numUnclaimedMeasurements;
            if (usedMeasurements != assignment.numCompletedMatches()) {
                std::cout
                    << HEAP_PREFIX
                    << "Error: numMeasurements - numUnclaimedMeasurements = "
                    << usedMeasurements << " but object reports "
                    << assignment.numCompletedMatches() << " matches!\n";
            }
            std::cout
                << HEAP_PREFIX
                << "Matched: " << assignment.numCompletedMatches()
                << "\tUnclaimed Meas: " << assignment.numUnclaimedMeasurements()
                << "\tUnclaimed LED: "
                << assignment.numUnclaimedLedObjects()
                /// this is how many elements the match-count early-out saved us
                << "\tRemaining: " << assignment.size() << "\n";
        }

        assignment.eraseUnclaimedLedObjects(verbose);

        // If we have any blobs that have not been associated with an
        // LED, then we add a new LED for each of them.
        // std::cout << "Had " << Leds.size() << " LEDs, " <<
        // keyPoints.size() << " new ones available" << std::endl;
        assignment.forEachUnclaimedMeasurement([&](LedMeasurement const &meas) {
            myLeds.emplace_back(m_impl->identifier.get(), meas);
        });

        /// Do the initial filtering of the LED group to just the identified
        /// ones before we pass it to an estimator.
        updateUsableLeds();

#ifdef UVBI_DUMP_BLOB_CSV
        {
            static bool first = true;
            if (first) {
                first = false;
                std::cout << "Dumping first row of blob data." << std::endl;
            }
            auto &row = m_impl->csv.row();
            for (auto &led : usableLeds()) {
                auto prefix =
                    std::to_string(led->getOneBasedID().value()) + ".";
                row << util::cellGroup(
                           prefix, cvToVector(led->getLocationForTracking()))
                    << util::cell(prefix + "diameter",
                                  led->getMeasurement().diameter)
                    << util::cell(prefix + "area", led->getMeasurement().area)
                    << util::cell(prefix + "bright", led->isBright());
            }
        }
#endif // UVBI_DUMP_BLOB_CSV

#ifdef UVBI_FRAMEDROP_HEURISTIC_WARNING
        if (usableLeds().empty() && prevUsableLedCount > 3 &&
            assignment.numCompletedMatches() > prevUsableLedCount / 2) {
            // if we don't have any usable LEDs, last time we had more than 3
            // (possibly not turning away), and this time we've got blobs that
            // match at least half of the blobs from last time.
            msg() << "WARNING: Likely dropped camera frame (reduce CPU load!): "
                     "no usable (identified) LEDs this frame out of "
                  << numMeasurements
                  << " detected likely beacons, while last frame contained "
                  << prevUsableLedCount << " usable LEDs." << std::endl;
        }
#endif

        return assignment.numCompletedMatches();
    }

    void TrackedBodyTarget::disableKalman() { m_impl->permitKalman = false; }

    void TrackedBodyTarget::permitKalman() { m_impl->permitKalman = true; }

    bool TrackedBodyTarget::updatePoseEstimateFromLeds(
        CameraParameters const &camParams,
        osvr::util::time::TimeValue const &tv, BodyState &bodyState,
        osvr::util::time::TimeValue const &startingTime,
        bool validStateAndTime) {

        /// Must pre/post correct the state by our offset :-/
        /// @todo make this state correction less hacky.
        bodyState.position() -= getStateCorrection();

        /// Will we permit Kalman this estimation?
        bool permitKalman = m_impl->permitKalman && validStateAndTime;

        /// OK, now must decide who we talk to for pose estimation.
        /// @todo move state machine logic elsewhere?

        if (!m_hasPoseEstimate && isStateSCAAT(m_impl->trackingState)) {
            /// Lost tracking somehow and we're in a SCAAT state.
            enterRANSACMode();
        }

        /// pre-estimation transitions based on overall health
        switch (m_impl->healthEval(bodyState, usableLeds(),
                                   m_impl->trackingState)) {
        case TargetHealthState::StopTrackingErrorBoundsExceeded: {
            msg() << "In flight reset - error bounds exceeded...";
#ifdef UVBI_VERBOSE_ERROR_BOUNDS
            auto maxPositionalError =
                getMaxPositionalErrorVariance(getBody().getState());
            auto distance = getBody().getState().position().z();
            auto errorLimit = getLimitOnMaxPositionalErrorVariance(distance);
            std::cout << " [" << maxPositionalError << "\t > " << errorLimit
                      << "\t (@ " << distance << "m)]";
#endif
            std::cout << std::endl;

            if (m_impl->softResets) {
                /// Smooth RANSAC here - we haven't lost all sight.
                enterRANSACKalmanMode();
            } else {
                enterRANSACMode();
            }
            break;
        }
        case TargetHealthState::StopTrackingLostSight:
#if 0
            msg() << "Lost sight of beacons for too long, awaiting their "
                     "return..."
                  << std::endl;
#endif
            enterRANSACMode();
            break;
        case TargetHealthState::HardResetNonFiniteState:
            msg() << "Hard reset - non-finite target state." << std::endl;
            enterRANSACMode();
            break;
        case TargetHealthState::OK:
            // we're ok, no transition needed.
            break;
        }
        /// Pre-estimation transitions per-state
        switch (m_impl->trackingState) {
        case TargetTrackingState::RANSACWhenBlobDetected: {
            if (!usableLeds().empty()) {
                msg()
                    << "In flight reset - beacons detected, re-acquiring fix..."
                    << std::endl;
                enterRANSACMode();
            }
            break;
        }

        case TargetTrackingState::RANSACKalmanWhenBlobDetected: {
            if (!usableLeds().empty()) {
                msg()
                    << "In flight reset - beacons detected, re-acquiring fix..."
                    << std::endl;
                enterRANSACKalmanMode();
            }
            break;
        }
        default:
            // other states don't have pre-estimation transitions.
            break;
        }

        /// main estimation dispatch
        auto params = EstimatorInOutParams{
            camParams, m_beacons, m_beaconMeasurementVariance, m_beaconFixed,
            m_beaconEmissionDirection, startingTime, bodyState,
            getBody().getProcessModel(), m_beaconDebugData,
            /*m_targetToBody*/
            Eigen::Vector3d::Zero()};
        switch (m_impl->trackingState) {
        case TargetTrackingState::RANSAC: {
            m_hasPoseEstimate = m_impl->ransacEstimator(params, usableLeds());
            m_impl->lastFrameAlgorithm = TargetTrackingState::RANSAC;
            break;
        }

        case TargetTrackingState::RANSACKalman: {
            m_hasPoseEstimate =
                m_impl->ransacKalmanEstimator(params, usableLeds(), tv);
            m_impl->lastFrameAlgorithm = TargetTrackingState::RANSACKalman;
            break;
        }

        case TargetTrackingState::RANSACWhenBlobDetected:
        case TargetTrackingState::EnteringKalman:
        case TargetTrackingState::Kalman: {
            auto videoDt =
                osvrTimeValueDurationSeconds(&tv, &m_impl->lastEstimate);
            m_hasPoseEstimate =
                m_impl->kalmanEstimator(params, usableLeds(), tv, videoDt);
            m_impl->lastFrameAlgorithm = TargetTrackingState::Kalman;
            break;
        }
        }

        /// End of main estimation dispatch

#ifdef UVBI_DEBUG_ERROR_VARIANCE

        static ::util::Stride varianceStride(101);
        if (++varianceStride) {
            msg() << "Max positional error variance: "
                  << getMaxPositionalErrorVariance(getBody().getState())
                  << "   Distance: " << getBody().getState().position().z()
                  << std::endl;
        }
#endif

        /// post-estimation transitions (based on state)
        switch (m_impl->trackingState) {
        case TargetTrackingState::RANSACKalman:
        case TargetTrackingState::RANSAC: {
            if (m_hasPoseEstimate && permitKalman) {
                enterKalmanMode();
            }
            break;
        }
        case TargetTrackingState::EnteringKalman:
            m_impl->trackingState = TargetTrackingState::Kalman;
            // Get one frame pass on the Kalman health check.
            break;
        case TargetTrackingState::Kalman: {
#ifndef UVBI_RANSACKALMAN
            auto health = m_impl->kalmanEstimator.getTrackingHealth();
            switch (health) {
            case SCAATKalmanPoseEstimator::TrackingHealth::NeedsHardResetNow:
                msg() << "In flight reset - lost fix..." << std::endl;
                enterRANSACMode();
                break;
            case SCAATKalmanPoseEstimator::TrackingHealth::
                SoftResetWhenBeaconsSeen:
#ifdef UVBI_DEBUG_ERROR_VARIANCE_WHEN_TRACKING_LOST
                msg() << "Max positional error variance: "
                      << getMaxPositionalErrorVariance(getBody().getState())
                      << std::endl;
#endif
                if (m_impl->softResets) {
                    m_impl->trackingState =
                        TargetTrackingState::RANSACKalmanWhenBlobDetected;
                } else {
                    m_impl->trackingState =
                        TargetTrackingState::RANSACWhenBlobDetected;
                }
                break;
            case SCAATKalmanPoseEstimator::TrackingHealth::Functioning:
                // OK!
                break;
            }
#endif
            break;
        }
        default:
            // other states don't have post-estimation transitions.
            break;
        }

        /// Update our local target-specific timestamp
        m_impl->lastEstimate = tv;

        /// Corresponding post-correction.
        bodyState.position() += getStateCorrection();

        return m_hasPoseEstimate;
    }

    bool TrackedBodyTarget::uncalibratedRANSACPoseEstimateFromLeds(
        CameraParameters const &camParams, Eigen::Vector3d &xlate,
        Eigen::Quaterniond &quat, int skipBrightsCutoff,
        std::size_t iterations) {

        Eigen::Vector3d outXlate;
        Eigen::Quaterniond outQuat;
        auto gotPose = m_impl->ransacEstimator(
            camParams, usableLeds(), m_beacons, m_beaconDebugData, outXlate,
            outQuat, skipBrightsCutoff, iterations);
        if (gotPose) {
            // Post-correct the state
            xlate = outXlate + computeTranslationCorrectionToBody(outQuat);
            // copy the quat
            quat = outQuat;
        }
        return gotPose;
    }

    Eigen::Vector3d TrackedBodyTarget::getStateCorrection() const {
// return m_impl->bodyInterface.state.getQuaternion().conjugate() *
// m_beaconOffset;
#if 0
        return computeTranslationCorrection(
            m_beaconOffset, m_impl->bodyInterface.state.getQuaternion());
#endif
        return computeTranslationCorrectionToBody(
            m_impl->bodyInterface.state.getQuaternion());
    }

    Eigen::Vector3d TrackedBodyTarget::computeTranslationCorrection(
        Eigen::Vector3d const &bodyFrameOffset,
        Eigen::Quaterniond const &orientation) {
        Eigen::Vector3d ret = (Eigen::Isometry3d(orientation) *
                               Eigen::Translation3d(bodyFrameOffset))
                                  .translation();
        return ret;
    }

    Eigen::Vector3d TrackedBodyTarget::computeTranslationCorrectionToBody(
        Eigen::Quaterniond const &orientation) const {
        return computeTranslationCorrection(m_beaconOffset + m_targetToBody,
                                            orientation);
    }

    ConfigParams const &TrackedBodyTarget::getParams() const {
        return m_body.getParams();
    }
    std::ostream &TrackedBodyTarget::msg() const {
        if (getParams().silent) {
            m_impl->outputSink.str("");
            return m_impl->outputSink;
        }
        return std::cout << "[Tracker Target " << getQualifiedId() << "] ";
    }
    void TrackedBodyTarget::enterKalmanMode() {
        msg() << "Entering SCAAT Kalman mode..." << std::endl;
        m_impl->trackingState = TargetTrackingState::EnteringKalman;
        m_impl->kalmanEstimator.resetCounters();
    }

    void TrackedBodyTarget::enterRANSACMode() {

#ifndef UVBI_ASSUME_SINGLE_TARGET_PER_BODY
#error                                                                         \
    "We may not be able/willing to run right over the body velocity just because this target lost its fix"
#endif
#ifdef UVBI_DEBUG_ERROR_VARIANCE_WHEN_TRACKING_LOST
        msg() << "Max positional error variance: "
              << getMaxPositionalErrorVariance(getBody().getState())
              << std::endl;
#endif
        m_impl->trackingResets++;
        // Zero out velocities if we're coming from Kalman.
        switch (m_impl->trackingState) {
        case TargetTrackingState::RANSACWhenBlobDetected:
        case TargetTrackingState::Kalman:
            getBody().getState().angularVelocity() = Eigen::Vector3d::Zero();
            getBody().getState().velocity() = Eigen::Vector3d::Zero();
            break;
        case TargetTrackingState::EnteringKalman:
            /// unlikely to have messed up velocity in one step. let it be.
            break;
        default:
            break;
        }
        m_impl->trackingState = TargetTrackingState::RANSAC;
    }

    void TrackedBodyTarget::enterRANSACKalmanMode() {
        /// Still counts as a reset.
        m_impl->trackingResets++;
#if 0
        // Zero out velocities if we're coming from Kalman.
        switch (m_impl->trackingState) {
        case TargetTrackingState::RANSACWhenBlobDetected:
        case TargetTrackingState::Kalman:
            getBody().getState().angularVelocity() = Eigen::Vector3d::Zero();
            getBody().getState().velocity() = Eigen::Vector3d::Zero();
            break;
        case TargetTrackingState::EnteringKalman:
            /// unlikely to have messed up velocity in one step. let it be.
            break;
        default:
            break;
        }
#endif
        msg() << "Soft reset as configured..." << std::endl;
        m_impl->trackingState = TargetTrackingState::RANSACKalman;
    }

    LedGroup const &TrackedBodyTarget::leds() const { return m_impl->leds; }

    LedPtrList const &TrackedBodyTarget::usableLeds() const {
        return m_impl->usableLeds;
    }

    std::size_t TrackedBodyTarget::numTrackingResets() const {
        return m_impl->trackingResets;
    }

    inline std::ptrdiff_t getNumUsedLeds(LedPtrList const &usableLeds) {
        return std::count_if(
            usableLeds.begin(), usableLeds.end(),
            [](Led const *ledPtr) { return ledPtr->wasUsedLastFrame(); });
    }

    double TrackedBodyTarget::getInternalStatusMeasurement(
        TargetStatusMeasurement measurement) const {
        switch (measurement) {
        case TargetStatusMeasurement::MaxPosErrorVariance:
            return getMaxPositionalErrorVariance(getBody().getState());
        case TargetStatusMeasurement::PosErrorVarianceLimit: {
            auto distance = getBody().getState().position().z();
            return getLimitOnMaxPositionalErrorVariance(distance);
        }
        case TargetStatusMeasurement::NumUsableLeds:
            return static_cast<double>(usableLeds().size());
        case TargetStatusMeasurement::NumUsedLeds:
            return static_cast<double>(getNumUsedLeds(usableLeds()));

        default:
            break;
        }
        return 0.0;
    }

    LedGroup &TrackedBodyTarget::leds() { return m_impl->leds; }

    LedPtrList &TrackedBodyTarget::usableLeds() { return m_impl->usableLeds; }
    void TrackedBodyTarget::updateUsableLeds() {
        auto &usable = usableLeds();
        usable.clear();
        auto &leds = m_impl->leds;
        for (auto &led : leds) {
            if (!led.identified()) {
                continue;
            }
            usable.push_back(&led);
        }
    }
    osvr::util::time::TimeValue const &
    TrackedBodyTarget::getLastUpdate() const {
        return m_impl->lastEstimate;
    }

    void TrackedBodyTarget::dumpBeaconsToConsole() const {

        /// Dump the beacon locations to console in a CSV-like format.
        auto numBeacons = getNumBeacons();
        std::cout << "BeaconsID,x,y,z" << std::endl;
        Eigen::IOFormat ourFormat(Eigen::StreamPrecision, 0, ",");
        for (UnderlyingBeaconIdType i = 0; i < numBeacons; ++i) {
            auto id = ZeroBasedBeaconId(i);
            std::cout << i + 1 << ","
                      << getBeaconAutocalibPosition(id).transpose().format(
                             ourFormat)
                      << "\n";
        }
    }

} // namespace vbtracker
} // namespace osvr
