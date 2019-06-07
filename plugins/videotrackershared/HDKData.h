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
#include "Types.h"

// Library/third-party includes
#include <opencv2/core/core.hpp>

// Standard includes
#include <cstddef>
#include <initializer_list>

namespace osvr {
namespace vbtracker {
    /// @name Default 3D locations for the beacons on an OSVR HDK 1.x chassis,
    /// in millimeters
    /// @{
    extern const Point3Vector OsvrHdkLedLocations_SENSOR0;
    extern const Point3Vector OsvrHdkLedLocations_SENSOR1;
    /// @}

    /// @name Default 3D locations for the beacons on an OSVR HDK2 chassis, in
    /// millimeters
    /// @{
    extern const Point3Vector OsvrHdk2LedLocations_SENSOR0;
    extern const Point3Vector OsvrHdk2LedLocations_SENSOR1;
    /// @}

    std::initializer_list<std::size_t> getOneBasedIDsOfMissingBeaconsHDK2();

    /// @name Emission directions for the beacons on an OSVR HDK
    /// @{
    extern const Vec3Vector OsvrHdkLedDirections_SENSOR0;
    extern const Vec3Vector OsvrHdkLedDirections_SENSOR1;
    /// @}

    extern const std::vector<double> OsvrHdkLedVariances_SENSOR0;

    inline std::size_t getNumHDKFrontPanelBeacons() {
        return OsvrHdkLedLocations_SENSOR0.size();
    }

    inline std::size_t getNumHDKRearPanelBeacons() {
        return OsvrHdkLedLocations_SENSOR1.size();
    }

    /// distance between front and back panel target origins, in mm.
    inline double
    computeDistanceBetweenPanels(double headCircumference,
                                 double headToFrontBeaconOriginDistance) {
        return headCircumference / CV_PI * 10. +
               headToFrontBeaconOriginDistance;
    }

} // namespace vbtracker
} // namespace osvr
