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

#ifndef INCLUDED_BodyIdTypes_h_GUID_4FD241A3_25E9_42AF_F075_8EE8191C02F0
#define INCLUDED_BodyIdTypes_h_GUID_4FD241A3_25E9_42AF_F075_8EE8191C02F0

// Internal Includes
// - none

// Library/third-party includes
#include <KalmanFramework/TypeSafeId.h>

// Standard includes
#include <cstdint>
#include <stdexcept>

namespace osvr {
namespace vbtracker {

    namespace detail {

        using BodyIdPolicy =
            flexkalman::util::TypeSafeIdMaxIntPolicy<std::uint16_t>;
        using TargetIdPolicy =
            flexkalman::util::TypeSafeIdMaxIntPolicy<std::uint8_t>;
    } // namespace detail

    using flexkalman::util::TypeSafeId;
    /// Type-safe zero-based body ID.
    using BodyId = TypeSafeId<struct BodyIdTag, detail::BodyIdPolicy>;
    /// Type-safe zero-based target ID.
    using TargetId = TypeSafeId<struct TargetIdTag, detail::TargetIdPolicy>;
    /// Type-safe zero-based target ID qualified with its body ID.
    using BodyTargetId = std::pair<BodyId, TargetId>;

    /// Stream output operator for the body-target ID.
    template <typename Stream>
    inline Stream &operator<<(Stream &os, BodyTargetId const &id) {
        os << id.first.value() << ":" << int(id.second.value());
        return os;
    }
} // namespace vbtracker
} // namespace osvr

namespace std {
template <> struct hash<osvr::vbtracker::BodyTargetId> {

    size_t operator()(const osvr::vbtracker::BodyTargetId &x) const {
        /// Using xor for safety internally, in case I got the shifts wrong,
        /// but it should be:
        /// [padding][bits from first element][bits from second element]
        std::uint32_t composed =
            (get(x.first) << sizeof(x.second)) ^ get(x.second);

        static_assert(sizeof(composed) <= sizeof(x),
                      "Should not lose data here!");
        return std::hash<std::uint32_t>{}(composed);
    }
};
} // namespace std

#endif // INCLUDED_BodyIdTypes_h_GUID_4FD241A3_25E9_42AF_F075_8EE8191C02F0
