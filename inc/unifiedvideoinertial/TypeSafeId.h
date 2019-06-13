/** @file
    @brief Header

    Based on OSVR header inc/osvr/Util/TypeSafeId.h
    but simplified to remove reference accessor
    and explicitly pass config type, instead

    @date 2015

    @author Sensics, Inc. <http://sensics.com/osvr>
    @author Ryan Pavlik <ryan.pavlik@collabora.com>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2018-2019 Collabora, Ltd.
//
// SPDX-License-Identifier: Apache-2.0
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
// - none

// Library/third-party includes
// - none

// Standard includes
#include <cstdint>
#include <functional>
#include <limits>

namespace videotracker {
namespace util {
    template <typename T> struct TypeSafeIdMaxIntPolicy {
        using type = T;
        static constexpr type sentinel_value =
            (std::numeric_limits<type>::max)();
    };
    using TypeSafeIdUint32Policy = TypeSafeIdMaxIntPolicy<std::uint32_t>;

    /// @brief A generic typesafe (as long as you use differing tag types)
    /// wrapper for identifiers (typically integers).
    ///
    /// @tparam Tag any type - does not have to be defined, just declared
    /// (so `struct mytag;` somewhere is fine). The tag serves to make integer
    /// IDs have distinct types.
    /// @tparam Policy A type providing a `type` typedef indicating the wrapped
    /// type, and a static constexpr sentinel_value, used for default "empty"
    /// construction.
    ///
    /// Initial implementation inspired by
    /// http://www.ilikebigbits.com/blog/2014/5/6/type-safe-identifiers-in-c
    /// though this version now strays quite far by strengthening
    /// type-safety and encapsulation, and by using traits classes to specify
    /// details based on tag type alone.
    template <typename Tag, typename Policy = TypeSafeIdUint32Policy>
    class TypeSafeId {
      public:
        //! The type of the current class.
        using type = TypeSafeId<Tag, Policy>;

        //! The contained/wrapped type.
        using wrapped_type = typename Policy::type;

        //! The sentinel value.
        static constexpr wrapped_type sentinel_value = Policy::sentinel_value;

        /*! Default constructor which will set value_ to the
         * sentinel (empty/invalid) value.
         */
        constexpr TypeSafeId() : value_(sentinel_value) {}

        //! Explicit constructor from the wrapped type
        constexpr explicit TypeSafeId(wrapped_type val) : value_(val) {}

        //! Check whether the ID is empty/invalid
        constexpr bool empty() const { return value_ == sentinel_value; }

        //! Check if ID is non-empty
        constexpr explicit operator bool() const { return !empty(); }

        //! Read-only accessor to the (non-type-safe!) wrapped value
        constexpr wrapped_type value() const { return value_; }

      private:
        //! Wrapped value
        wrapped_type value_;
    };
    /*! Find out if a TypeSafeId if empty/invalid.
     *
     * @relates ::videotracker::util::TypeSafeId
     */
    template <typename Tag, typename Policy>
    inline constexpr bool is_empty(TypeSafeId<Tag, Policy> const id) noexcept {
        return id.empty();
    }

    /*! Get the (type-unsafe) wrapped value of a TypeSafeId value.
     *
     * If the ID is empty, you will get the sentinel value back.
     *
     * Can be found through argument-dependent lookup, in theory. I think
     * this means you could use it unqualified.
     *
     * @relates ::videotracker::util::TypeSafeId
     */
    template <typename Tag, typename Policy>
    inline constexpr typename Policy::type
    get(TypeSafeId<Tag, Policy> const id) noexcept {
        return id.value();
    }

    /*! @brief Equality comparison operator for type-safe IDs.
     *
     * @relates ::videotracker::util::TypeSafeId
     */
    template <typename Tag, typename Policy>
    inline constexpr bool operator==(TypeSafeId<Tag, Policy> const a,
                                     TypeSafeId<Tag, Policy> const b) {
        return get(a) == get(b);
    }

    /*! @brief Inequality comparison operator for type-safe IDs.
     *
     * @relates ::videotracker::util::TypeSafeId
     */
    template <typename Tag, typename Policy>
    inline constexpr bool operator!=(TypeSafeId<Tag, Policy> const a,
                                     TypeSafeId<Tag, Policy> const b) {
        return get(a) != get(b);
    }
    /*! @brief Less-than comparison operator for type-safe IDs.
     *
     * @relates ::videotracker::util::TypeSafeId
     */
    template <typename Tag, typename Policy>
    inline constexpr bool operator<(TypeSafeId<Tag, Policy> const a,
                                    TypeSafeId<Tag, Policy> const b) {
        return get(a) < get(b);
    }

} // namespace util
} // namespace videotracker

namespace std {
template <typename Tag, typename Policy>
struct hash<::videotracker::util::TypeSafeId<Tag, Policy>> {
    using Type = ::videotracker::util::TypeSafeId<Tag, Policy>;
    using WrappedType = typename Type::wrapped_type;
    size_t operator()(const Type &x) const {
        return std::hash<WrappedType>{}(get(x));
    }
};
} // namespace std
