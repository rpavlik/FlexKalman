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
#include "List.h"
#include "T.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace osvr {
namespace typepack {

    /// @brief Will turn whatever is passed into it into the simplest list.
    template <typename... Ts> using coerce_list = t_<list<Ts...>>;

} // namespace typepack
} // namespace osvr
