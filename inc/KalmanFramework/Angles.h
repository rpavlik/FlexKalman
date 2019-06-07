/** @file
    @brief Header

    @date 2019

    @author Ryan Pavlik <ryan.pavlik@collabora.com>
*/

// Copyright 2019 Collabora, Ltd.
//
// SPDX-License-Identifier: BSL-1.0

#pragma once

namespace flexkalman {
namespace util {

    template <typename Scalar> struct AngleRadians {
      public:
        using value_type = Scalar;
        AngleRadians() : val(0) {}
        explicit AngleRadians(value_type x) : val(x) {}
        AngleRadians(AngleRadians const &) = default;
        AngleRadians &operator=(AngleRadians const &) = default;

        value_type value() const { return val; }

      private:
        value_type val;
    };

    using AngleRadiansd = AngleRadians<double>;

    /// @brief Default angle type
    using Angle = AngleRadiansd;

    /// @brief Get the raw scalar value of your angle in radians
    template <typename Scalar>
    inline Scalar getRadians(AngleRadians<Scalar> const angle) {
        return angle.value();
    }

    /// @brief Get the raw scalar value of your angle in degrees
    template <typename Scalar>
    inline Scalar getDegrees(AngleRadians<Scalar> const angle) {
        return angle.value() * Scalar(180) / EIGEN_PI;
    }

} // namespace util
} // namespace flexkalman
