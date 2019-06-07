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
// 	http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Internal Includes
#include "LedIdentifier.h"
#include "Types.h"

// Library/third-party includes
// - none

// Standard includes
#include <stdint.h>

namespace osvr {
namespace vbtracker {

    /// @brief Factory function to create an HDK Led Identifier object
    /// @param sensor either 0 (front plate) or 1 (back plate)
    LedIdentifierPtr createHDKLedIdentifier(uint8_t sensor);

    /// @brief Factory function to create an HDK Led Identifier object
    ///        using the presumed order that generated the simulated
    ///        images, which turned out to be changed before release.
    /// @param sensor either 0 (front plate) or 1 (back plate)
    LedIdentifierPtr createHDKLedIdentifierSimulated(uint8_t sensor);

    /// @brief Factory function to create an HDK Led Identifier object
    /// containing all the beacons in order for the front plate followed by the
    /// back plate.
    LedIdentifierPtr createHDKUnifiedLedIdentifier();

    /// @brief Factory function to create an HDK Led Identifier object using the
    /// random images patterns.
    LedIdentifierPtr createRandomHDKLedIdentifier();
} // namespace vbtracker
} // namespace osvr
