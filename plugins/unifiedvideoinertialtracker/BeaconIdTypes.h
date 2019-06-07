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
#include "BodyIdTypes.h"

// Library/third-party includes
#include "UVBIAssert.h"
#include <KalmanFramework/TypeSafeId.h>

// Standard includes
#include <stdexcept>

namespace osvr {
namespace vbtracker {
    /// All beacon IDs, whether 0 or 1 based, are ints on the inside.
    using UnderlyingBeaconIdType = int;
    namespace detail {
        using BeaconIdPolicy =
            flexkalman::util::TypeSafeIdMaxIntPolicy<UnderlyingBeaconIdType>;
    } // namespace detail

    using flexkalman::util::TypeSafeId;
    /// Type-safe zero-based beacon ID.
    using ZeroBasedBeaconId =
        TypeSafeId<struct ZeroBasedBeaconIdTag, detail::BeaconIdPolicy>;
    /// Type-safe one-based beacon ID.
    using OneBasedBeaconId =
        TypeSafeId<struct OneBasedBeaconIdTag, detail::BeaconIdPolicy>;

    /// Overloaded conversion function to turn any beacon ID into one-based,
    /// respecting the convention that negative values don't change.
    inline OneBasedBeaconId makeOneBased(ZeroBasedBeaconId id) {
        OneBasedBeaconId ret;
        if (id.empty()) {
            return ret;
        } else if (id.value() < 0) {
            ret = OneBasedBeaconId(id.value());
        } else {
            ret = OneBasedBeaconId(id.value() + 1);
        }
        return ret;
    }

    /// No-op overload, so you can take any beacon ID and ensure it is
    /// one-based.
    inline OneBasedBeaconId const &makeOneBased(OneBasedBeaconId const &id) {
        return id;
    }

    /// Overloaded conversion function to turn any beacon ID into zero-based,
    /// respecting the convention that negative values don't change.
    inline ZeroBasedBeaconId makeZeroBased(OneBasedBeaconId id) {
        ZeroBasedBeaconId ret;
        if (id.empty()) {
            return ret;
        } else if (id.value() < 0) {
            ret = ZeroBasedBeaconId(id.value());
        } else {
            ret = ZeroBasedBeaconId(id.value() - 1);
        }
        return ret;
    }

    /// No-op overload, so you can take any beacon ID and ensure it is
    /// zero-based.
    inline ZeroBasedBeaconId const &makeZeroBased(ZeroBasedBeaconId const &id) {
        return id;
    }

    /// Does the given beacon ID indicate that it's identified?
    inline bool beaconIdentified(ZeroBasedBeaconId id) {
        return (!id.empty() && id.value() >= 0);
    }
    /// Does the given beacon ID indicate that it's identified?
    inline bool beaconIdentified(OneBasedBeaconId id) {
        return (!id.empty() && id.value() > 0);
    }

    /// Turn a (valid non-sentinel, i.e. identified) beacon id into an array
    /// index.
    inline std::size_t asIndex(ZeroBasedBeaconId id) {
        UVBI_ASSERT_MSG(beaconIdentified(id), "A beacon id must correspond to "
                                              "an identified beacon to be "
                                              "used as an index!");
        if (!beaconIdentified(id)) {
            throw std::logic_error("A beacon id must correspond to an "
                                   "identified beacon to be used as an index!");
        }
        return id.value();
    }

    /// @overload
    inline std::size_t asIndex(OneBasedBeaconId id) {
        return asIndex(makeZeroBased(id));
    }

} // namespace vbtracker
} // namespace osvr
