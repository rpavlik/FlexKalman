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
#include "Apply.h"
#include "List.h"
#include "T.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace osvr {
namespace typepack {

    namespace detail {

        /// The trait used to implement the alias typepack::transform
        template <typename List, typename Fun> struct transform_;
        template <typename... Ts, typename Fun>
        struct transform_<list<Ts...>, Fun> {
            /// required for MSVC2013 to avoid "there are no parameter packs
            /// available to expand"
            template <typename T> struct apply_workaround {
                using type = typepack::apply<Fun, T>;
            };
            using type = list<typename apply_workaround<Ts>::type...>;
        };
    } // namespace detail

    /// Given a list and an alias class, apply the alias class to each element
    /// in the list and return the results in a list.
    template <typename List, typename Fun>
    using transform = t_<detail::transform_<List, Fun>>;
} // namespace typepack
} // namespace osvr
