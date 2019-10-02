/** @file
    @brief Header for measurements of absolute orientation.

    @date 2015

    @author
    Ryan Pavlik
    <ryan.pavlik@collabora.com>
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
#include "BaseTypes.h"
#include "EigenQuatExponentialMap.h"
#include "ExternalQuaternion.h"
#include "FlexibleKalmanBase.h"
#include "PoseState.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {
/*!
 * A measurement of absolute orientation in 3D space.
 *
 * It can be used with any state class that exposes a `getCombinedQuaternion()`
 * method. On its own, it is only suitable for unscented filter correction,
 * since the jacobian depends on the arrangement of the state vector. See
 * AbsoluteOrientationEKFMeasurement's explicit specializations for use in EKF
 * correction mode.
 */
class AbsoluteOrientationMeasurement {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    AbsoluteOrientationMeasurement(Eigen::Quaterniond const &quat,
                                   types::Vector<3> const &emVariance)
        : m_quat(quat), m_covariance(emVariance.asDiagonal()) {}

    template <typename State>
    MeasurementSquareMatrix const &getCovariance(State const &) {
        return m_covariance;
    }

    /*!
     * Gets the measurement residual, also known as innovation: predicts
     * the measurement from the predicted state, and returns the
     * difference.
     *
     * State type doesn't matter as long as we can
     * `.getCombinedQuaternion()`
     */
    template <typename State>
    MeasurementVector getResidual(State const &s) const {
        const Eigen::Quaterniond prediction = s.getCombinedQuaternion();
        // Two equivalent quaternions: but their logs are typically
        // different: one is the "short way" and the other is the "long
        // way". We'll compute both and pick the "short way".
        return 2 * util::smallest_quat_ln(m_quat * prediction.conjugate());
    }
    //! Convenience method to be able to store and re-use measurements.
    void setMeasurement(Eigen::Quaterniond const &quat) { m_quat = quat; }

    /*!
     * Get the block of jacobian that is non-zero: your subclass will have
     * to put it where it belongs for each particular state type.
     */
    types::Matrix<Dimension, 3> getJacobianBlock() const {
        return Eigen::Matrix3d::Identity();
    }

  private:
    Eigen::Quaterniond m_quat;
    MeasurementSquareMatrix m_covariance;
};

/*!
 * This is the subclass of AbsoluteOrientationMeasurement: only explicit
 * specializations, and on state types.
 *
 * Only required for EKF-style correction (since jacobian depends closely on the
 * state).
 */
template <typename StateType> class AbsoluteOrientationEKFMeasurement;

//! AbsoluteOrientationEKFMeasurement with a pose_externalized_rotation::State
template <>
class AbsoluteOrientationEKFMeasurement<pose_externalized_rotation::State>
    : public AbsoluteOrientationMeasurement,
      public MeasurementBase<AbsoluteOrientationEKFMeasurement<
          pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t StateDimension = getDimension<State>();
    using Base = AbsoluteOrientationMeasurement;

    AbsoluteOrientationEKFMeasurement(Eigen::Quaterniond const &quat,
                                      types::Vector<3> const &eulerVariance)
        : Base(quat, eulerVariance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &s) const {
        using namespace pose_externalized_rotation;
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.block<Dimension, 3>(0, 3) = Base::getJacobianBlock();
        return ret;
    }
};

} // namespace flexkalman
