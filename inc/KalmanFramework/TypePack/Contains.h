/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// TypePack is part of OSVR-Core.
//
// Use, modification and distribution is subject to the
// Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#pragma once

// Internal Includes
#include "ApplyList.h"
#include "Or.h"
#include "Quote.h"
#include "Transform.h"

// Library/third-party includes
// - none

// Standard includes
#include <type_traits>

namespace osvr {
namespace typepack {

    namespace detail {
        /// Bind the first argument of std::is_same
        template <typename T> struct is_ {
            template <typename Elt> using apply = std::is_same<T, Elt>;
        };

    } // namespace detail

    /// @brief Determines if type @p Needle is in the list @p Haystack - is an
    /// alias for a type that inherits std::true_type or std::false_type.
    template <typename Haystack, typename Needle>
    using contains =
        apply_list<quote<or_>, transform<Haystack, detail::is_<Needle>>>;

} // namespace typepack
} // namespace osvr
