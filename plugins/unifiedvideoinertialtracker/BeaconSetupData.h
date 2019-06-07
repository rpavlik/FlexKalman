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
#include "BeaconIdTypes.h"
#include "Types.h"

// Library/third-party includes
#include <opencv2/core/core.hpp>

// Standard includes
#include <string>
#include <vector>

namespace osvr {
namespace vbtracker {
    using EmissionDirectionVec = ::cv::Vec3d;
    using LocationPoint = ::cv::Point3f;

    struct TargetDataSummary {
        std::vector<OneBasedBeaconId> disabledByPattern;
        std::vector<OneBasedBeaconId> disabledByEmptyPattern;
        std::vector<std::pair<OneBasedBeaconId, std::string>> errors;
        std::vector<OneBasedBeaconId> validBeacons;
    };

    /// Data for a full target (all the beacons), unswizzled into a "struct of
    /// vectors". All should be the same size, since they are parallel.
    struct TargetSetupData {
        std::vector<std::string> patterns;
        Point3Vector locations;
        Vec3Vector emissionDirections;
        std::vector<double> baseMeasurementVariances;
        std::vector<double> initialAutocalibrationErrors;
        std::vector<bool> isFixed;

        using size_type = std::vector<double>::size_type;

        size_type numBeacons() const { return locations.size(); }

        /// This is both an entirely unlikely out of bounds value and a
        /// specific sentinel value.
        static LocationPoint getBogusLocation() {
            return LocationPoint(-10000.f, -10000.f, -87314159.f);
        }
        /// Resizes all arrays to the numBeacons.
        /// Only populates baseMeasurementVariances,
        /// initialAutocalibrationErrors, and isFixed
        /// with semi-reasonable default values (no beacons fixed)
        void setBeaconCount(std::size_t numBeacons,
                            double baseMeasurementVariance,
                            double initialAutocalibrationError) {
            patterns.resize(numBeacons);
            locations.resize(numBeacons, getBogusLocation());
            // these are invalid directions and must be populated!
            emissionDirections.resize(numBeacons,
                                      EmissionDirectionVec(0, 0, 0));
            baseMeasurementVariances.resize(numBeacons,
                                            baseMeasurementVariance);
            initialAutocalibrationErrors.resize(numBeacons,
                                                initialAutocalibrationError);
            isFixed.resize(numBeacons, false);
        }

        /// Mark a beacon, by zero-based ID, as being fixed.
        void markBeaconFixed(ZeroBasedBeaconId beacon) {
            isFixed.at(beacon.value()) = true;
            initialAutocalibrationErrors.at(beacon.value()) = 0;
        }
        /// Mark a beacon, by one-based ID, as being fixed.
        void markBeaconFixed(OneBasedBeaconId beacon) {
            markBeaconFixed(makeZeroBased(beacon));
        }

        /// Is the beacon active?
        bool isBeaconActive(OneBasedBeaconId beacon);

        void markBeaconInactive(ZeroBasedBeaconId beacon);
        void markBeaconInactive(OneBasedBeaconId beacon) {
            markBeaconInactive(makeZeroBased(beacon));
        }

        TargetDataSummary cleanAndValidate(bool silent = false);
    };

    /// Output operator for a target data summary.
    template <typename Stream>
    inline Stream &operator<<(Stream &os, TargetDataSummary const &summary) {
        os << "\n\nTarget Data Summary:\n";
        os << "\nBeacons disabled by their pattern:\n";
        for (auto id : summary.disabledByPattern) {
            os << id.value() << "\n";
        }
        os << "\nBeacons disabled by empty pattern:\n";
        for (auto id : summary.disabledByEmptyPattern) {
            os << id.value() << "\n";
        }
        os << "\nBeacons with errors:\n";
        for (auto &err : summary.errors) {
            os << err.first.value() << ": " << err.second << "\n";
        }
        os << "\nValid beacons:\n";
        for (auto id : summary.validBeacons) {
            os << id.value() << " ";
        }
        os << "\n\n";
        return os;
    }
} // namespace vbtracker
} // namespace osvr
