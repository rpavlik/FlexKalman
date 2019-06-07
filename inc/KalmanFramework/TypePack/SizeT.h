/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>

    @author
    Eric Niebler
*/

// Copyright 2015 Sensics, Inc.
// TypePack is part of OSVR-Core.
//
// Incorporates code from "meta":
// Copyright Eric Niebler 2014-2015
//
// Use, modification and distribution is subject to the
// Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Project home: https://github.com/ericniebler/meta
//

#pragma once

// Internal Includes
// - none

// Library/third-party includes
// - none

// Standard includes
#include <cstddef>
#include <type_traits>

namespace osvr {
namespace typepack {
    /// @brief Alias template to simplify creating an integral constant of
    /// size_t
    template <std::size_t V>
    using size_t_ = std::integral_constant<std::size_t, V>;
} // namespace typepack
} // namespace osvr
