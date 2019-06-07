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
#include "BlobParams.h"

// Library/third-party includes
// - none

// Standard includes
#include <cstdint>
#include <string>

namespace osvr {
namespace vbtracker {

    static const double BaseMeasurementVariance = 3.0;

    struct IMUInputParams {
        std::string path =
            "/com_osvr_Multiserver/OSVRHackerDevKit0/semantic/hmd";

        /// Should we use the IMU orientation data for calibration even if
        /// useOrientation is false?
        bool calibrateAnyway = false;

        /// Should orientation reports be used once calibration completes?
        bool useOrientation = true;

        /// units: rad^2
        double orientationVariance = 1.0e-7;

        std::int32_t orientationMicrosecondsOffset = 0;

        /// Should angular velocity reports be used once calibration completes?
        bool useAngularVelocity = true;

        /// units: (rad/sec)^2
        double angularVelocityVariance = 1.0e-1;

        std::int32_t angularVelocityMicrosecondsOffset = 0;
    };

    struct TuningParams {
        TuningParams();
        double noveltyPenaltyBase;

        double distanceMeasVarianceBase;
        double distanceMeasVarianceIntercept;
    };

    /// If you add an entry here, must also update both
    /// getConfigStringForTargetSet and AllBuiltInTargetSets in
    /// ConfigurationParser.h
    enum class BuiltInTargetSets { HDK1xChassis, HDK2Chassis };

    /// General configuration parameters
    struct ConfigParams {
        /// Not intended to be manually configurable - enabled when doing things
        /// like running an optimization algorithm so some things like a debug
        /// view might need to change.
        bool performingOptimization = false;

        /// For optimization usage.
        bool silent = false;

        /// For recording tuning data - whether we should record the raw blob
        /// data.
        bool logRawBlobs = false;

        /// For recording tuning data - whether we should record the data from
        /// just the usable LEDs each frame after they're associated.
        bool logUsableLeds = false;

        TuningParams tuning;

        /// Parameters specific to the blob-detection step of the algorithm
        BlobParams blobParams;

        /// Parameters specific to the edge hole based LED extraction algorithm.
        EdgeHoleParams extractParams;

        /// When using hard-coded target sets, which one to use.
        BuiltInTargetSets targetSet = BuiltInTargetSets::HDK1xChassis;

        /// Should we have the tracking thread update the reporting vector for
        /// every (IMU) message, instead of waiting/buffering for a few
        /// milliseconds between updates?
        bool continuousReporting = true;

        /// Should we open the camera in high-gain mode?
        bool highGain = true;

        /// Seconds beyond the current time to predict, using the Kalman state.
        double additionalPrediction = 0.;

        /// Max residual, in meters at the expected XY plane of the beacon in
        /// space, for a beacon before applying a variance penalty.
        double maxResidual = 0.03631354168383816;

        /// Initial beacon error for autocalibration (units: m^2).
        /// 0 effectively turns off beacon auto-calib.
        /// This is a variance number, so std deviation squared, but it's
        /// pretty likely to be between 0 and 1, so the variance will be smaller
        /// than the standard deviation.
        double initialBeaconError = 1e-7; // 0.001;

        /// Maximum distance a blob can move, in multiples of its previous
        /// "keypoint diameter", and still be considered the same blob.
        double blobMoveThreshold = 3.5;

        /// Whether to show the debug windows and debug messages.
        bool debug = false;

        /// How many threads to let OpenCV use. Set to 0 or less to let OpenCV
        /// decide (that is, not set an explicit preference)
        int numThreads = 1;

        /// This is the autocorrelation kernel of the process noise. The first
        /// three elements correspond to position, the second three to
        /// incremental rotation.
        double processNoiseAutocorrelation[6];

        /// The value used in exponential decay of linear velocity: it's the
        /// proportion of that velocity remaining at the end of 1 second. Thus,
        /// smaller = faster decay/higher damping. In range [0, 1]
        double linearVelocityDecayCoefficient = 0.9040551503451977;

        /// The value used in exponential decay of angular velocity: it's the
        /// proportion of that velocity remaining at the end of 1 second. Thus,
        /// smaller = faster decay/higher damping. In range [0, 1]
        double angularVelocityDecayCoefficient = 0.8945437897688864;

        /// The value used in an additional exponential decay of linear velocity
        /// when we've lost sight of all beacons, to quickly attenuate coasting.
        /// it's the proportion of that velocity remaining at the end of 1
        /// second. Thus, smaller = faster decay/higher damping. In range [0, 1]
        double noBeaconLinearVelocityDecayCoefficient = 0.005878868009089861;

        /// The measurement variance (units: m^2) is included in the plugin
        /// along with the coordinates of the beacons. Some beacons are
        /// observed with higher variance than others, due to known difficulties
        /// in tracking them, etc. However, for testing you may fine-tine the
        /// measurement variances globally by scaling them here.
        double measurementVarianceScaleFactor = 1.5;

        /// Whether the tracking algorithm internally adjusts beacon positions
        /// based on the centroid of the input beacon positions.
        bool offsetToCentroid = false;

        /// Manual beacon offset (in m) - only really sensible if you only have
        /// one target, only used if offsetToCentroid is false.
        double manualBeaconOffset[3];

        /// If true, this will replace the two sensors with just a single one,
        /// including the beacons at the back of the head "rigidly" as a part of
        /// it. If true, recommend offsetToCentroid = false, and
        /// manualBeaconOffset to be 0, 0, -75.
        bool includeRearPanel = true;

        /// Head circumference at the head strap, in cm - 55.75 is our estimate
        /// for an average based on some hat sizing guidelines. Only matters if
        /// includeRearPanel is true.
        double headCircumference = 55.75;

        /// This is the distance fron the front of the head to the origin of the
        /// front sensor coordinate system in the Z axis, in mm.
        /// This is a rough estimate - the origin of the coordinate system is
        /// roughly the flat part of the hard plastic.
        double headToFrontBeaconOriginDistance = 0;

        /// This used to be different than the other beacons, but now it's
        /// mostly the same.
        double backPanelMeasurementError = BaseMeasurementVariance;

        /// This is the process-model noise in the beacon-auto-calibration, in
        /// mm^2/s. Not fully accurate, since it only gets applied when a beacon
        /// gets used for a measurement, but it should be enough to keep beacons
        /// from converging in a bad local minimum.
        double beaconProcessNoise = 1.e-19;

        /// This is the multiplicative penalty applied to the variance of
        /// measurements with a "bad" residual
        double highResidualVariancePenalty = 7.513691210865344;

        /// When true, will stream debug info (variance, pixel measurement,
        /// pixel residual) on up to the first 34 beacons of your first sensor
        /// as analogs.
        bool streamBeaconDebugInfo = false;

        /// This should be the ratio of lengths of sides that you'll permit to
        /// be filtered in. Larger side first, please.
        ///
        /// Not currently being used.
        float boundingBoxFilterRatio = 5.f / 4.f;

        /// This should be a negative number - it's the largest the z component
        /// of the camera-space LED emission vector is permitted to be and still
        /// be used in estimation. acos(this number) is the maximum angle away
        /// from pointing at the camera that we'll accept an LED pointing.
        double maxZComponent = -0.3;

        /// Should we attempt to skip bright-mode LEDs? The alternative is to
        /// just give them slightly higher variance.
        bool shouldSkipBrightLeds = false;

        /// If shouldSkipBrightLeds is false, we use this value as a factor to
        /// increase the measurement variance of bright LEDs, to account for the
        /// fact that they are less accurate because they tend to refract
        /// through surrounding materials, etc.
        double brightLedVariancePenalty = 28.32749811268542;

        /// If this option is set to true, then while some of the pattern
        /// identifier is run each frame, an "early-out" will be taken if the
        /// blob/LED already has a valid (non-negative) ID assigned to it. This
        /// can help keep IDs on hard to identify blobs, but it can also persist
        /// errors longer. That's why it's an option.
        ///
        /// Defaulting to off because it adds some jitter for some reason.
        bool blobsKeepIdentity = false;

        /// Extra verbose developer debugging messages
        bool extraVerbose = false;

        /// If non-empty, the file to load (or save to) for calibration data.
        /// Only make sense for a single target.
        std::string calibrationFile = "";

        /// IMU input-related parameters.
        IMUInputParams imu;

        /// x, y, z, with y up, all in meters.
        double cameraPosition[3];

        /// Whether we should adjust transforms to assume the camera looks along
        /// the YZ plane in the +Z direction.
        bool cameraIsForward = true;

        /// Should we permit the whole system to enter Kalman mode? Not doing so
        /// is usually a bad idea, unless you're doing something special like
        /// development on the tracker itself...
        bool permitKalman = true;

        /// Time offset for the camera timestamp, in microseconds.
        /// Default is measured on Windows 10 version 1511.
        std::int32_t cameraMicrosecondsOffset = -27000;

        /// Should we permit a reset to be "soft" (blended by a Kalman) rather
        /// than a hard state setting, in certain conditions? Only available in
        /// the Unified tracker.
        bool softResets = false;

        /// Soft reset data incorporation parameter: Positional variance scale -
        /// multiplied by the square of the distance from the camera.
        double softResetPositionVarianceScale = 1.e-1;

        /// Soft reset data incorporation parameter: Orientation variance
        double softResetOrientationVariance = 1.e0;

        ConfigParams();
    };
} // namespace vbtracker
} // namespace osvr
