/** @file
    @brief Generated single-file header containing all of the FlexKalman flexible Kalman filter
   framework headers


    NOTE: This is a generated single-file version of FlexKalman - do not edit directly!
    Instead, edit the individual source files and regenerate this with combine_headers.py.

    @date 2015-2019

    @author
    Ryan Pavlik
    <ryan.pavlik@collabora.com>

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>

*/

// Copyright 2015-2016 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
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
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <cstddef>
#include <functional>
#include <type_traits>

#ifndef FLEXKALMAN_DEBUG_OUTPUT
#define FLEXKALMAN_DEBUG_OUTPUT(Name, Value)
#endif

/*!
 * @brief Header-only framework for building Kalman-style filters, prediction,
 * and sensor fusion
 */
namespace flexkalman {

//! @brief Type aliases, including template type aliases.
namespace types {
    //! Common scalar type
    using Scalar = double;
} // namespace types

/*!
 * Convenience base class for things (like states and measurements) that
 * have a dimension.
 */
template <size_t DIM> struct HasDimension {
    static constexpr size_t Dimension = DIM;
};

template <typename T> static constexpr size_t getDimension() {
    return T::Dimension;
}

namespace types {
    //! Given a filter type, get the state type.
    template <typename FilterType> using StateType = typename FilterType::State;

    //! Given a filter type, get the process model type.
    template <typename FilterType>
    using ProcessModelType = typename FilterType::ProcessModel;

    //! A vector of length n
    template <size_t n> using Vector = Eigen::Matrix<Scalar, n, 1>;

    //! A square matrix, n x n
    template <size_t n> using SquareMatrix = Eigen::Matrix<Scalar, n, n>;

    //! A square diagonal matrix, n x n
    template <size_t n> using DiagonalMatrix = Eigen::DiagonalMatrix<Scalar, n>;

    //! A matrix with rows = m,  cols = n
    template <size_t m, size_t n> using Matrix = Eigen::Matrix<Scalar, m, n>;

    //! A matrix with rows = dimension of T, cols = dimension of U
    template <typename T, typename U>
    using DimMatrix = Matrix<T::Dimension, U::Dimension>;

} // namespace types

/*!
 * Computes P-
 *
 * Usage is optional, most likely called from the process model
 * `updateState()`` method.
 */
template <typename StateType, typename ProcessModelType>
inline types::SquareMatrix<getDimension<StateType>()>
predictErrorCovariance(StateType const &state, ProcessModelType &processModel,
                       double dt) {
    using StateSquareMatrix = types::SquareMatrix<getDimension<StateType>()>;
    StateSquareMatrix A = processModel.getStateTransitionMatrix(state, dt);
    // FLEXKALMAN_DEBUG_OUTPUT("State transition matrix", A);
    StateSquareMatrix P = state.errorCovariance();
    /*!
     * @todo Determine if the fact that Q is (at least in one case)
     * symmetrical implies anything else useful performance-wise here or
     * later in the data flow.
     */
    // auto Q = processModel.getSampledProcessNoiseCovariance(dt);
    FLEXKALMAN_DEBUG_OUTPUT("Process Noise Covariance Q",
                            processModel.getSampledProcessNoiseCovariance(dt));
    return A * P * A.transpose() +
           processModel.getSampledProcessNoiseCovariance(dt);
}


template <typename Derived> class StateBase;
template <typename Derived> class MeasurementBase;
template <typename Derived> class ProcessModelBase;

template <typename StateType, typename MeasurementType>
struct CorrectionInProgress {
    //! Dimension of measurement
    static constexpr size_t m = getDimension<MeasurementType>();
    //! Dimension of state
    static constexpr size_t n = getDimension<StateType>();

    CorrectionInProgress(StateType &state, MeasurementType &meas,
                         types::SquareMatrix<n> const &P_,
                         types::Matrix<n, m> const &PHt_,
                         types::SquareMatrix<m> const &S)
        : P(P_), PHt(PHt_), denom(S), deltaz(meas.getResidual(state)),
          stateCorrection(PHt * denom.solve(deltaz)), state_(state),
          stateCorrectionFinite(stateCorrection.array().allFinite()) {}

    //! State error covariance
    types::SquareMatrix<n> P;

    //! The kalman gain stuff to not invert (called P12 in TAG)
    types::Matrix<n, m> PHt;

    /*!
     * Decomposition of S
     *
     * Not going to directly compute Kalman gain K = PHt (S^-1)
     * Instead, decomposed S to solve things of the form (S^-1)x
     * repeatedly later, by using the substitution
     * Kx = PHt denom.solve(x)
     * @todo Figure out if this is the best decomp to use
     */
    // TooN/TAG use this one, and others online seem to suggest it.
    Eigen::LDLT<types::SquareMatrix<m>> denom;

    //! Measurement residual/delta z/innovation
    types::Vector<m> deltaz;

    //! Corresponding state change to apply.
    types::Vector<n> stateCorrection;

    //! Is the state correction free of NaNs and +- infs?
    bool stateCorrectionFinite;

    //! That's as far as we go here before you choose to continue.

    /*!
     * Finish computing the rest and correct the state.
     *
     * @param cancelIfNotFinite If the new error covariance is detected to
     * contain non-finite values, should we cancel the correction and not
     * apply it?
     *
     * @return true if correction completed
     */
    bool finishCorrection(bool cancelIfNotFinite = true) {
        // Compute the new error covariance
        // differs from the (I-KH)P form by not factoring out the P (since
        // we already have PHt computed).
        types::SquareMatrix<n> newP = P - (PHt * denom.solve(PHt.transpose()));

#if 0
        // Test fails with this one:
        // VariedProcessModelStability/1.AbsolutePoseMeasurementXlate111,
        // where TypeParam =
        // flexkalman::PoseDampedConstantVelocityProcessModel
        FLEXKALMAN_DEBUG_OUTPUT(
            "error covariance scale",
            (types::SquareMatrix<n>::Identity() - PHt * denom.solve(H)));
        types::SquareMatrix<n> newP =
            (types::SquareMatrix<n>::Identity() - PHt * denom.solve(H)) * P;
#endif

        if (!newP.array().allFinite()) {
            return false;
        }

        // Correct the state estimate
        state_.setStateVector(state_.stateVector() + stateCorrection);

        // Correct the error covariance
        state_.setErrorCovariance(newP);

#if 0
        // Doesn't seem necessary to re-symmetrize the covariance matrix.
        state_.setErrorCovariance((newP + newP.transpose()) / 2.);
#endif

        // Let the state do any cleanup it has to (like fixing externalized
        // quaternions)
        state_.postCorrect();
        return true;
    }

  private:
    StateType &state_;
};

template <typename State, typename ProcessModel, typename Measurement>
inline CorrectionInProgress<State, Measurement>
beginExtendedCorrection(StateBase<State> &state,
                        ProcessModelBase<ProcessModel> &processModel,
                        MeasurementBase<Measurement> &meas) {

    //! Dimension of measurement
    static constexpr size_t m = getDimension<Measurement>();
    //! Dimension of state
    static constexpr size_t n = getDimension<State>();

    //! Measurement Jacobian
    types::Matrix<m, n> H = meas.derived().getJacobian(state.derived());

    //! Measurement covariance
    types::SquareMatrix<m> R = meas.derived().getCovariance(state.derived());

    //! State error covariance
    types::SquareMatrix<n> P = state.derived().errorCovariance();

    //! The kalman gain stuff to not invert (called P12 in TAG)
    types::Matrix<n, m> PHt = P * H.transpose();

    /*!
     * the stuff to invert for the kalman gain
     * also sometimes called S or the "Innovation Covariance"
     */
    types::SquareMatrix<m> S = H * PHt + R;

    //! More computation is done in initializers/constructor
    return {state.derived(), meas.derived(), P, PHt, S};
}

/*!
 * Correct a Kalman filter's state using a measurement that provides a
 * Jacobian, in the manner of an Extended Kalman Filter (EKF).
 *
 * @param cancelIfNotFinite If the state correction or new error covariance
 * is detected to contain non-finite values, should we cancel the
 * correction and not apply it?
 *
 * @return true if correction completed
 */
template <typename State, typename ProcessModel, typename Measurement>
static inline bool correctExtended(StateBase<State> &state,
                                   ProcessModelBase<ProcessModel> &processModel,
                                   MeasurementBase<Measurement> &meas,
                                   bool cancelIfNotFinite = true) {

    auto inProgress = beginExtendedCorrection(state, processModel, meas);
    if (cancelIfNotFinite && !inProgress.stateCorrectionFinite) {
        return false;
    }

    return inProgress.finishCorrection(cancelIfNotFinite);
}

//! Delegates to correctExtended, a more explicit name which is preferred.
template <typename State, typename ProcessModel, typename Measurement>
static inline bool
correct(StateBase<State> &state, ProcessModelBase<ProcessModel> &processModel,
        MeasurementBase<Measurement> &meas, bool cancelIfNotFinite = true) {
    return correctExtended(state, processModel, meas, cancelIfNotFinite);
}


template <typename Derived> class StateBase;
template <typename Derived> class MeasurementBase;
template <typename Derived> class ProcessModelBase;

/*!
 * Advance time in the filter, by applying the process model to the state with
 * the given dt.
 *
 * Usually followed by correct() or beginUnscentedCorrection() and its
 * continuation. If you aren't correcting immediately, make sure to run
 * `state.postCorrect()` to clean up. Otherwise, consider calling
 * getPrediction() instead.
 */
template <typename StateType, typename ProcessModelType>
static inline void predict(StateBase<StateType> &state,
                           ProcessModelBase<ProcessModelType> &processModel,
                           double dt) {
    processModel.derived().predictState(state.derived(), dt);
    FLEXKALMAN_DEBUG_OUTPUT("Predicted state",
                            state.derived().stateVector().transpose());

    FLEXKALMAN_DEBUG_OUTPUT("Predicted error covariance",
                            state.derived().errorCovariance());
}

/*!
 * Performs prediction of the state only (not the error covariance), as well as
 * post-correction. Unsuitable for continued correction for this reason, but
 * usable to get a look at a predicted value.
 *
 * Requires that your process model provide `processModel.predictStateOnly()`
 */
template <typename StateType, typename ProcessModelType>
static inline void
predictAndPostCorrectStateOnly(StateBase<StateType> &state,
                               ProcessModelBase<ProcessModelType> &processModel,
                               double dt) {
    processModel.derived().predictStateOnly(state.derived(), dt);
    state.derived().postCorrect();
    FLEXKALMAN_DEBUG_OUTPUT("Predicted state",
                            state.derived().stateVector().transpose());
}

/*!
 * Performs prediction on a copy of the state, as well as
 * post-correction. By default, also skips prediction of error covariance.
 *
 * Requires that your process model provide `processModel.predictStateOnly()`
 */
template <typename StateType, typename ProcessModelType>
static inline StateType
getPrediction(StateBase<StateType> const &state,
              ProcessModelBase<ProcessModelType> const &processModel, double dt,
              bool predictCovariance = false) {
    StateType stateCopy{state.derived()};
    if (predictCovariance) {
        processModel.derived().predictState(stateCopy, dt);
    } else {
        processModel.derived().predictStateOnly(stateCopy, dt);
    }
    stateCopy.postCorrect();
    return stateCopy;
}


/*!
 * @brief CRTP base for state types.
 *
 * All your State types should derive from this template, passing themselves as
 * the Derived type.
 */
template <typename Derived> class StateBase {
  public:
    Derived &derived() noexcept { return *static_cast<Derived *>(this); }
    Derived const &derived() const noexcept {
        return *static_cast<Derived const *>(this);
    }
};

/*!
 * @brief CRTP base for measurement types.
 *
 * All your Measurement types should derive from this template, passing
 * themselves as the Derived type.
 */
template <typename Derived> class MeasurementBase {
  public:
    Derived &derived() noexcept { return *static_cast<Derived *>(this); }
    Derived const &derived() const noexcept {
        return *static_cast<Derived const *>(this);
    }
};

/*!
 * @brief CRTP base for process model types.
 *
 * All your ProcessModel types should derive from this template, passing
 * themselves as the Derived type.
 */
template <typename Derived> class ProcessModelBase {
  public:
    Derived &derived() noexcept { return *static_cast<Derived *>(this); }
    Derived const &derived() const noexcept {
        return *static_cast<Derived const *>(this);
    }
};
namespace util {
    namespace ei_quat_exp_map {

        template <typename Scalar> struct FourthRootMachineEps;
        template <> struct FourthRootMachineEps<double> {
            //! machine epsilon is 1e-53, so fourth root is roughly 1e-13
            static double get() { return 1.e-13; }
        };
        template <> struct FourthRootMachineEps<float> {
            //! machine epsilon is 1e-24, so fourth root is 1e-6
            static float get() { return 1.e-6f; }
        };
        /*!
         * Computes the "historical" (un-normalized) sinc(Theta)
         * (sine(theta)/theta for theta != 0, defined as the limit value of 0
         * at theta = 0)
         */
        template <typename Scalar> inline Scalar sinc(Scalar theta) {
            /*!
             * fourth root of machine epsilon is recommended cutoff for taylor
             * series expansion vs. direct computation per
             * Grassia, F. S. (1998). Practical Parameterization of Rotations
             * Using the Exponential Map. Journal of Graphics Tools, 3(3),
             * 29-48. http://doi.org/10.1080/10867651.1998.10487493
             */
            Scalar ret;
            if (theta < FourthRootMachineEps<Scalar>::get()) {
                // taylor series expansion.
                ret = Scalar(1.f) - theta * theta / Scalar(6.f);
                return ret;
            }
            // direct computation.
            ret = std::sin(theta) / theta;
            return ret;
        }

        //! fully-templated free function for quaternion expontiation
        template <typename Derived>
        inline Eigen::Quaternion<typename Derived::Scalar>
        quat_exp(Eigen::MatrixBase<Derived> const &vec) {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
            using Scalar = typename Derived::Scalar;
            /*!
             * Implementation inspired by
             * Grassia, F. S. (1998). Practical Parameterization of Rotations
             * Using the Exponential Map. Journal of Graphics Tools, 3(3),
             * 29–48. http://doi.org/10.1080/10867651.1998.10487493
             *
             * However, that work introduced a factor of 1/2 which I could not
             * derive from the definition of quaternion exponentiation and
             * whose absence thus distinguishes this implementation. Without
             * that factor of 1/2, the exp and ln functions successfully
             * round-trip and match other implementations.
             */
            Scalar theta = vec.norm();
            Scalar vecscale = sinc(theta);
            Eigen::Quaternion<Scalar> ret;
            ret.vec() = vecscale * vec;
            ret.w() = std::cos(theta);
            return ret.normalized();
        }

        /*!
         * fully-templated free function for "small-angle" approximation of
         * quaternion expontiation
         */
        template <typename Derived>
        inline Eigen::Quaternion<typename Derived::Scalar>
        small_angle_quat_exp(Eigen::MatrixBase<Derived> const &vec) {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
            using Scalar = typename Derived::Scalar;
            //! @todo get better way of determining "zero vec" for this purpose.
            if (vec.isApproxToConstant(0, 1.e-4)) {
                return Eigen::Quaternion<Scalar>::Identity();
            }
            // For non-zero vectors, the vector scale sinc(theta) approximately
            // equals 1, while the scale for w, cos(theta), is approximately 0.
            // So, we treat this as a "pure" quaternion and normalize it.
            return Eigen::Quaternion<Scalar>{0, vec.x(), vec.y(), vec.z()}
                .normalized();
        }

        /*!
         * Taylor series expansion of theta over sin(theta), aka cosecant, for
         * use near 0 when you want continuity and validity at 0.
         */
        template <typename Scalar>
        inline Scalar cscTaylorExpansion(Scalar theta) {
            return Scalar(1) +
                   // theta ^ 2 / 6
                   (theta * theta) / Scalar(6) +
                   // 7 theta^4 / 360
                   (Scalar(7) * theta * theta * theta * theta) / Scalar(360) +
                   // 31 theta^6/15120
                   (Scalar(31) * theta * theta * theta * theta * theta *
                    theta) /
                       Scalar(15120);
        }

        /*!
         * fully-templated free function for quaternion log map.
         *
         * Assumes a unit quaternion.
         *
         * @todo seems to be off by a factor of two in testing?
         */
        template <typename Scalar>
        inline Eigen::Matrix<Scalar, 3, 1>
        quat_ln(Eigen::Quaternion<Scalar> const &quat) {
            // ln q = ( (phi)/(norm of vec) vec, ln(norm of quat))
            // When we assume a unit quaternion, ln(norm of quat) = 0
            // so then we just scale the vector part by phi/sin(phi) to get the
            // result (i.e., ln(qv, qw) = (phi/sin(phi)) * qv )
            Scalar vecnorm = quat.vec().norm();

            // "best for numerical stability" vs asin or acos
            Scalar phi = std::atan2(vecnorm, quat.w());

            // Here is where we compute the coefficient to scale the vector part
            // by, which is nominally phi / std::sin(phi).
            // When the angle approaches zero, we compute the coefficient
            // differently, since it gets a bit like sinc in that we want it
            // continuous but 0 is undefined.
            Scalar phiOverSin = vecnorm < 1e-4 ? cscTaylorExpansion<Scalar>(phi)
                                               : (phi / std::sin(phi));
            return quat.vec() * phiOverSin;
        }

        /*!
         * Takes the smallest of two equivalent quat logarithms.
         *
         * The quaternions are equivalent, but their logarithms are often
         * different, so we choose the "shortest one". Often used for angular
         * residuals.
         */
        static inline Eigen::Vector3d
        smallest_quat_ln(Eigen::Quaterniond const &q) {
            //! @todo optimize - to avoid duplicating sign-invariant parts of
            //! quat_ln
            Eigen::Vector3d v = quat_ln(q);
            Eigen::Vector3d equiv = quat_ln(Eigen::Quaterniond(-(q.coeffs())));

            return v.squaredNorm() < equiv.squaredNorm() ? v : equiv;
        }
    } // namespace ei_quat_exp_map
    using ei_quat_exp_map::quat_exp;
    using ei_quat_exp_map::quat_ln;
    using ei_quat_exp_map::small_angle_quat_exp;
    using ei_quat_exp_map::smallest_quat_ln;
} // namespace util

namespace external_quat {
    /*!
     * For use in maintaining an "external quaternion" and 3 incremental
     * orientations, as done by Welch based on earlier work.
     *
     * Performs exponentiation from a vector to a quaternion.
     */
    inline Eigen::Quaterniond vecToQuat(types::Vector<3> const &incRotVec) {
        return util::quat_exp(incRotVec / 2.);
    }
/*!
 * Computes what is effectively the Jacobian matrix of partial
 * derivatives of incrementalOrientationToQuat()
 */
#if 0
        inline types::Matrix<4, 3> jacobian(Eigen::Vector3d const &incRotVec) {
            assert(vecToQuatScalarPartSquared(incRotVec) >= 0 &&
                   "Incremental rotation vector's squared norm was greater "
                   "than 1! Precondition fail!");
            // eigen internally stores quaternions x, y, z, w
            types::Matrix<4, 3> ret;
            // vector components of jacobian are all 1/2 identity
            ret.topLeftCorner<3, 3>() =
                types::SquareMatrix<3>::Identity() * 0.5;
            ret.bottomRows<1>() =
                incRotVec.transpose() /
                (-4. * std::sqrt(vecToQuatScalarPartSquared(incRotVec)));
            return ret;
        }
#endif
#if 0
        inline types::Matrix<4, 3> jacobian(Eigen::Vector3d const &w) {
            double a = w.squaredNorm() / 48 + 0.5;
            // outer product over 24, plus a on the diagonal
            Eigen::Matrix3d topBlock =
                (w * w.transpose()) / 24. + Eigen::Matrix3d::Identity() * a;
            // this weird thing on the bottom row.
            Eigen::RowVector3d bottomRow =
                (Eigen::Vector3d(2 * a, 0, 0) + (w[0] * w) / 12 - w / 4)
                    .transpose();
            types::Matrix<4, 3> ret;
            ret << topBlock, bottomRow;
            return ret;
        }
#endif
} // namespace external_quat


namespace pose_externalized_rotation {
    constexpr size_t Dimension = 12;
    using StateVector = types::Vector<Dimension>;
    using StateVectorBlock3 = StateVector::FixedSegmentReturnType<3>::Type;
    using ConstStateVectorBlock3 =
        StateVector::ConstFixedSegmentReturnType<3>::Type;

    using StateVectorBlock6 = StateVector::FixedSegmentReturnType<6>::Type;
    using ConstStateVectorBlock6 =
        StateVector::ConstFixedSegmentReturnType<6>::Type;
    using StateSquareMatrix = types::SquareMatrix<Dimension>;

    /*!
     * This returns A(deltaT), though if you're just predicting xhat-, use
     * applyVelocity() instead for performance.
     */
    inline StateSquareMatrix stateTransitionMatrix(double dt) {
        // eq. 4.5 in Welch 1996 - except we have all the velocities at the
        // end
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<6, 6>() = types::SquareMatrix<6>::Identity() * dt;

        return A;
    }
    /*!
     * Function used to compute the coefficient m in v_new = m * v_old.
     * The damping value is for exponential decay.
     */
    inline double computeAttenuation(double damping, double dt) {
        return std::pow(damping, dt);
    }

    /*!
     * Returns the state transition matrix for a constant velocity with a
     * single damping parameter (not for direct use in computing state
     * transition, because it is very sparse, but in computing other
     * values)
     */
    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(double dt, double damping) {
        // eq. 4.5 in Welch 1996
        auto A = stateTransitionMatrix(dt);
        A.bottomRightCorner<6, 6>() *= computeAttenuation(damping, dt);
        return A;
    }

    /*!
     * Returns the state transition matrix for a constant velocity with
     * separate damping paramters for linear and angular velocity (not for
     * direct use in computing state transition, because it is very sparse,
     * but in computing other values)
     */
    inline StateSquareMatrix stateTransitionMatrixWithSeparateVelocityDamping(
        double dt, double posDamping, double oriDamping) {
        // eq. 4.5 in Welch 1996
        auto A = stateTransitionMatrix(dt);
        A.block<3, 3>(6, 6) *= computeAttenuation(posDamping, dt);
        A.bottomRightCorner<3, 3>() *= computeAttenuation(oriDamping, dt);
        return A;
    }

    class State : public StateBase<State> {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static constexpr size_t Dimension = 12;

        //! Default constructor
        State()
            : m_state(StateVector::Zero()),
              m_errorCovariance(StateSquareMatrix::Identity() *
                                10 /** @todo almost certainly wrong */),
              m_orientation(Eigen::Quaterniond::Identity()) {}
        //! set xhat
        void setStateVector(StateVector const &state) { m_state = state; }
        //! xhat
        StateVector const &stateVector() const { return m_state; }

        // set P
        void setErrorCovariance(StateSquareMatrix const &errorCovariance) {
            m_errorCovariance = errorCovariance;
        }
        //! P
        StateSquareMatrix const &errorCovariance() const {
            return m_errorCovariance;
        }
        StateSquareMatrix &errorCovariance() { return m_errorCovariance; }

        //! Intended for startup use.
        void setQuaternion(Eigen::Quaterniond const &quaternion) {
            m_orientation = quaternion.normalized();
        }

        void postCorrect() { externalizeRotation(); }

        void externalizeRotation() {
            setQuaternion(getCombinedQuaternion());
            incrementalOrientation() = Eigen::Vector3d::Zero();
        }

        StateVectorBlock3 position() { return m_state.head<3>(); }

        ConstStateVectorBlock3 position() const { return m_state.head<3>(); }

        StateVectorBlock3 incrementalOrientation() {
            return m_state.segment<3>(3);
        }

        ConstStateVectorBlock3 incrementalOrientation() const {
            return m_state.segment<3>(3);
        }

        StateVectorBlock3 velocity() { return m_state.segment<3>(6); }

        ConstStateVectorBlock3 velocity() const {
            return m_state.segment<3>(6);
        }

        StateVectorBlock3 angularVelocity() { return m_state.segment<3>(9); }

        ConstStateVectorBlock3 angularVelocity() const {
            return m_state.segment<3>(9);
        }

        //! Linear and angular velocities
        StateVectorBlock6 velocities() { return m_state.tail<6>(); }

        //! Linear and angular velocities
        ConstStateVectorBlock6 velocities() const { return m_state.tail<6>(); }

        Eigen::Quaterniond const &getQuaternion() const {
            return m_orientation;
        }

        Eigen::Quaterniond getCombinedQuaternion() const {
            // divide by 2 since we're integrating it essentially.
            return util::quat_exp(incrementalOrientation() / 2.) *
                   m_orientation;
        }

        /*!
         * Get the position and quaternion combined into a single isometry
         * (transformation)
         */
        Eigen::Isometry3d getIsometry() const {
            Eigen::Isometry3d ret;
            ret.fromPositionOrientationScale(position(), getQuaternion(),
                                             Eigen::Vector3d::Constant(1));
            return ret;
        }

      private:
        /*!
         * In order: x, y, z, incremental rotations phi (about x), theta
         * (about y), psy (about z), then their derivatives in the same
         * order.
         */
        StateVector m_state;
        //! P
        StateSquareMatrix m_errorCovariance;
        //! Externally-maintained orientation per Welch 1996
        Eigen::Quaterniond m_orientation;
    };

    /*!
     * Stream insertion operator, for displaying the state of the state
     * class.
     */
    template <typename OutputStream>
    inline OutputStream &operator<<(OutputStream &os, State const &state) {
        os << "State:" << state.stateVector().transpose() << "\n";
        os << "quat:" << state.getCombinedQuaternion().coeffs().transpose()
           << "\n";
        os << "error:\n" << state.errorCovariance() << "\n";
        return os;
    }

    //! Computes A(deltaT)xhat(t-deltaT)
    inline void applyVelocity(State &state, double dt) {
        // eq. 4.5 in Welch 1996

        /*!
         * @todo benchmark - assuming for now that the manual small
         * calcuations are faster than the matrix ones.
         */

        state.position() += state.velocity() * dt;
        state.incrementalOrientation() += state.angularVelocity() * dt;
    }

    //! Dampen all 6 components of velocity by a single factor.
    inline void dampenVelocities(State &state, double damping, double dt) {
        auto attenuation = computeAttenuation(damping, dt);
        state.velocities() *= attenuation;
    }

    //! Separately dampen the linear and angular velocities
    inline void separatelyDampenVelocities(State &state, double posDamping,
                                           double oriDamping, double dt) {
        state.velocity() *= computeAttenuation(posDamping, dt);
        state.angularVelocity() *= computeAttenuation(oriDamping, dt);
    }
} // namespace pose_externalized_rotation


class AbsoluteOrientationMeasurementBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    AbsoluteOrientationMeasurementBase(Eigen::Quaterniond const &quat,
                                       types::Vector<3> const &emVariance)
        : m_quat(quat), m_covariance(emVariance.asDiagonal()) {}

    template <typename State>
    MeasurementSquareMatrix const &getCovariance(State const &) {
        return m_covariance;
    }

    template <typename State>
    MeasurementVector predictMeasurement(State const &state) const {
        return state.incrementalOrientation();
    }
    template <typename State>
    MeasurementVector getResidual(MeasurementVector const &predictedMeasurement,
                                  State const &s) const {
        // The prediction we're given is effectively "the state's incremental
        // rotation", which is why we're using our measurement here as well as
        // the prediction.
        const Eigen::Quaterniond fullPredictedOrientation =
            util::small_angle_quat_exp(predictedMeasurement / 2.) *
            s.getQuaternion();
        return 2 * util::smallest_quat_ln(m_quat *
                                          fullPredictedOrientation.conjugate());
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

  private:
    Eigen::Quaterniond m_quat;
    MeasurementSquareMatrix m_covariance;
};
/*!
 * A measurement of absolute orientation in 3D space.
 *
 * It can be used with any state class that exposes a `getCombinedQuaternion()`
 * method (that is, an externalized quaternion state). On its own, it is only
 * suitable for unscented filter correction, since the jacobian depends on the
 * arrangement of the state vector. See AbsoluteOrientationEKFMeasurement's
 * explicit specializations for use in EKF correction mode.
 */
class AbsoluteOrientationMeasurement
    : public AbsoluteOrientationMeasurementBase,
      public MeasurementBase<AbsoluteOrientationMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using AbsoluteOrientationMeasurementBase::
        AbsoluteOrientationMeasurementBase;
};

/*!
 * This is the EKF-specific relative of AbsoluteOrientationMeasurement: only
 * explicit specializations, and on state types.
 *
 * Only required for EKF-style correction (since jacobian depends closely on the
 * state).
 */
template <typename StateType> class AbsoluteOrientationEKFMeasurement;

//! AbsoluteOrientationEKFMeasurement with a pose_externalized_rotation::State
template <>
class AbsoluteOrientationEKFMeasurement<pose_externalized_rotation::State>
    : public AbsoluteOrientationMeasurementBase,
      public MeasurementBase<AbsoluteOrientationEKFMeasurement<
          pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    static constexpr size_t StateDimension = getDimension<State>();
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AbsoluteOrientationEKFMeasurement(Eigen::Quaterniond const &quat,
                                      types::Vector<3> const &eulerVariance)
        : AbsoluteOrientationMeasurementBase(quat, eulerVariance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &s) const {
        using namespace pose_externalized_rotation;
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.block<Dimension, 3>(0, 3) = types::SquareMatrix<3>::Identity();
        return ret;
    }
};


//! A constant-velocity model for a 6DOF pose (with velocities)
class PoseConstantVelocityProcessModel
    : public ProcessModelBase<PoseConstantVelocityProcessModel> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = pose_externalized_rotation::State;
    using StateVector = pose_externalized_rotation::StateVector;
    using StateSquareMatrix = pose_externalized_rotation::StateSquareMatrix;
    using NoiseAutocorrelation = types::Vector<6>;
    PoseConstantVelocityProcessModel(double positionNoise = 0.01,
                                     double orientationNoise = 0.1) {
        setNoiseAutocorrelation(positionNoise, orientationNoise);
    }
    void setNoiseAutocorrelation(double positionNoise = 0.01,
                                 double orientationNoise = 0.1) {
        m_mu.head<3>() = types::Vector<3>::Constant(positionNoise);
        m_mu.tail<3>() = types::Vector<3>::Constant(orientationNoise);
    }
    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_mu = noise;
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &, double dt) const {
        return pose_externalized_rotation::stateTransitionMatrix(dt);
    }

    //! Does not update error covariance
    void predictStateOnly(State &s, double dt) const {
        FLEXKALMAN_DEBUG_OUTPUT("Time change", dt);
        pose_externalized_rotation::applyVelocity(s, dt);
    }
    //! Updates state vector and error covariance
    void predictState(State &s, double dt) const {
        predictStateOnly(s, dt);
        auto Pminus = predictErrorCovariance(s, *this, dt);
        s.setErrorCovariance(Pminus);
    }

    /*!
     * This is Q(deltaT) - the Sampled Process Noise Covariance
     * @return a matrix of dimension n x n.
     *
     * Like all covariance matrices, it is real symmetrical (self-adjoint),
     * so .selfAdjointView<Eigen::Upper>() might provide useful performance
     * enhancements in some algorithms.
     */
    StateSquareMatrix getSampledProcessNoiseCovariance(double dt) const {
        constexpr auto dim = getDimension<State>();
        StateSquareMatrix cov = StateSquareMatrix::Zero();
        auto dt3 = (dt * dt * dt) / 3;
        auto dt2 = (dt * dt) / 2;
        for (std::size_t xIndex = 0; xIndex < dim / 2; ++xIndex) {
            auto xDotIndex = xIndex + dim / 2;
            // xIndex is 'i' and xDotIndex is 'j' in eq. 4.8
            const auto mu = getMu(xIndex);
            cov(xIndex, xIndex) = mu * dt3;
            auto symmetric = mu * dt2;
            cov(xIndex, xDotIndex) = symmetric;
            cov(xDotIndex, xIndex) = symmetric;
            cov(xDotIndex, xDotIndex) = mu * dt;
        }
        return cov;
    }

  private:
    /*!
     * this is mu-arrow, the auto-correlation vector of the noise
     * sources
     */
    NoiseAutocorrelation m_mu;
    double getMu(std::size_t index) const {
        assert(index < (getDimension<State>() / 2) &&
               "Should only be passing "
               "'i' - the main state, not "
               "the derivative");
        // This may not be totally correct but it's one of the parameters
        // you can kind of fudge in kalman filters anyway.
        // Should techincally be the diagonal of the correlation kernel of
        // the noise sources. (p77, p197 in Welch 1996)
        return m_mu(index);
    }
};


/*!
 * A basically-constant-velocity model, with the addition of some
 * damping of the velocities inspired by TAG. This model has separate
 * damping/attenuation of linear and angular velocities.
 */
class PoseSeparatelyDampedConstantVelocityProcessModel
    : public ProcessModelBase<
          PoseSeparatelyDampedConstantVelocityProcessModel> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = pose_externalized_rotation::State;
    using StateVector = pose_externalized_rotation::StateVector;
    using StateSquareMatrix = pose_externalized_rotation::StateSquareMatrix;
    using BaseProcess = PoseConstantVelocityProcessModel;
    using NoiseAutocorrelation = BaseProcess::NoiseAutocorrelation;
    PoseSeparatelyDampedConstantVelocityProcessModel(
        double positionDamping = 0.3, double orientationDamping = 0.01,
        double positionNoise = 0.01, double orientationNoise = 0.1)
        : m_constantVelModel(positionNoise, orientationNoise) {
        setDamping(positionDamping, orientationDamping);
    }

    void setNoiseAutocorrelation(double positionNoise = 0.01,
                                 double orientationNoise = 0.1) {
        m_constantVelModel.setNoiseAutocorrelation(positionNoise,
                                                   orientationNoise);
    }

    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_constantVelModel.setNoiseAutocorrelation(noise);
    }
    //! Set the damping - must be in (0, 1)
    void setDamping(double posDamping, double oriDamping) {
        if (posDamping > 0 && posDamping < 1) {
            m_posDamp = posDamping;
        }
        if (oriDamping > 0 && oriDamping < 1) {
            m_oriDamp = oriDamping;
        }
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &, double dt) const {
        return pose_externalized_rotation::
            stateTransitionMatrixWithSeparateVelocityDamping(dt, m_posDamp,
                                                             m_oriDamp);
    }

    void predictStateOnly(State &s, double dt) const {
        m_constantVelModel.predictStateOnly(s, dt);
        // Dampen velocities
        pose_externalized_rotation::separatelyDampenVelocities(s, m_posDamp,
                                                               m_oriDamp, dt);
    }
    void predictState(State &s, double dt) const {
        predictStateOnly(s, dt);
        auto Pminus = predictErrorCovariance(s, *this, dt);
        s.setErrorCovariance(Pminus);
    }

    /*!
     * This is Q(deltaT) - the Sampled Process Noise Covariance
     * @return a matrix of dimension n x n. Note that it is real
     * symmetrical (self-adjoint), so .selfAdjointView<Eigen::Upper>()
     * might provide useful performance enhancements.
     */
    StateSquareMatrix getSampledProcessNoiseCovariance(double dt) const {
        return m_constantVelModel.getSampledProcessNoiseCovariance(dt);
    }

  private:
    BaseProcess m_constantVelModel;
    double m_posDamp = 0.2;
    double m_oriDamp = 0.01;
};


inline double computeNu(std::size_t L, double alpha) {
    auto lambda = (L * (alpha * alpha - 1));
    auto nu = std::sqrt(L + lambda);
    return nu;
}
/*!
 * For further details on the scaling factors, refer to:
 * Julier, S. J., & Uhlmann, J. K. (2004). Unscented filtering and
 * nonlinear estimation. Proceedings of the IEEE, 92(3), 401�422.
 * http://doi.org/10.1109/JPROC.2003.823141
 * Appendix V (for alpha), Appendix VI (for beta)
 */
struct SigmaPointParameters {
    SigmaPointParameters(double alpha_ = 0.001, double beta_ = 2.,
                         double kappa_ = 0.)
        : alpha(alpha_), beta(beta_), kappa(kappa_) {}
    /*!
     * double L;
     * Primary scaling factor, typically in the range [1e-4, 1]
     */
    double alpha;
    /*!
     * Secondary scaling to emphasize the 0th sigma point in covariance
     * weighting - 2 is optimal for gaussian distributions
     */
    double beta;
    /*!
     * Tertiary scaling factor, typically 0.
     * Some authors recommend parameter estimation to use L - 3
     */
    double kappa;
};
struct SigmaPointParameterDerivedQuantities {
    SigmaPointParameterDerivedQuantities(SigmaPointParameters const &p,
                                         std::size_t L)
        : alphaSquared(p.alpha * p.alpha),
          lambda(alphaSquared * (L + p.kappa) - L),
          gamma(std::sqrt(L + lambda)), weightMean0(lambda / (L + lambda)),
          weightCov0(weightMean0 + p.beta + 1 - alphaSquared),
          weight(1. / (2. * (L + lambda))) {}

  private:
    double alphaSquared;

  public:
    //! "Compound scaling factor"
    double lambda;
    //! Scales the matrix square root in computing sigma points
    double gamma;

    //! Element 0 of weight vector for computing means
    double weightMean0;
    //! Element 0 of weight vector for computing covariance
    double weightCov0;
    //! Other elements of weight vector
    double weight;
};

template <std::size_t Dim, std::size_t OrigDim = Dim>
class AugmentedSigmaPointGenerator {
  public:
    static_assert(OrigDim <= Dim, "Original, non-augmented dimension must "
                                  "be equal or less than the full "
                                  "dimension");
    static const std::size_t L = Dim;
    static const std::size_t OriginalDimension = OrigDim;
    static const std::size_t NumSigmaPoints = L * 2 + 1;
    using MeanVec = types::Vector<Dim>;
    using CovMatrix = types::SquareMatrix<Dim>;
    using SigmaPointsMat = types::Matrix<Dim, NumSigmaPoints>;
    using SigmaPointWeightVec = types::Vector<NumSigmaPoints>;

    AugmentedSigmaPointGenerator(MeanVec const &mean, CovMatrix const &cov,
                                 SigmaPointParameters params)
        : p_(params, L), mean_(mean), covariance_(cov) {
        weights_ = SigmaPointWeightVec::Constant(p_.weight);
        weightsForCov_ = weights_;
        weights_[0] = p_.weightMean0;
        weightsForCov_[0] = p_.weightCov0;
        scaledMatrixSqrt_ = cov.llt().matrixL();
        //! scaledMatrixSqrt_ *= p_.gamma;
        sigmaPoints_ << mean, (p_.gamma * scaledMatrixSqrt_).colwise() + mean,
            (-p_.gamma * scaledMatrixSqrt_).colwise() + mean;
    }

    SigmaPointsMat const &getSigmaPoints() const { return sigmaPoints_; }

    using SigmaPointBlock = Eigen::Block<SigmaPointsMat, OrigDim, 1>;
    using ConstSigmaPointBlock = Eigen::Block<const SigmaPointsMat, OrigDim, 1>;

    ConstSigmaPointBlock getSigmaPoint(std::size_t i) const {
        return sigmaPoints_.template block<OrigDim, 1>(0, i);
    }

    SigmaPointWeightVec const &getWeightsForMean() const { return weights_; }
    SigmaPointWeightVec const &getWeightsForCov() const {
        return weightsForCov_;
    }

    MeanVec const &getMean() const { return mean_; }

    using ConstOrigMeanVec = Eigen::VectorBlock<const MeanVec, OrigDim>;

    //! Get the "un-augmented" mean
    ConstOrigMeanVec getOrigMean() const {
        return mean_.template head<OrigDim>();
    }

  private:
    SigmaPointParameterDerivedQuantities p_;
    MeanVec mean_;
    CovMatrix covariance_;
    CovMatrix scaledMatrixSqrt_;
    SigmaPointsMat sigmaPoints_;
    SigmaPointWeightVec weights_;
    SigmaPointWeightVec weightsForCov_;
};

template <std::size_t Dim>
using SigmaPointGenerator = AugmentedSigmaPointGenerator<Dim, Dim>;

template <std::size_t XformedDim, typename SigmaPointsGenType>
class ReconstructedDistributionFromSigmaPoints {
  public:
    static const std::size_t Dimension = XformedDim;
    using SigmaPointsGen = SigmaPointsGenType;
    static const std::size_t NumSigmaPoints = SigmaPointsGen::NumSigmaPoints;

    static const size_t OriginalDimension = SigmaPointsGen::OriginalDimension;
    using TransformedSigmaPointsMat = types::Matrix<XformedDim, NumSigmaPoints>;

    using CrossCovMatrix = types::Matrix<OriginalDimension, Dimension>;

    using MeanVec = types::Vector<XformedDim>;
    using CovMat = types::SquareMatrix<XformedDim>;
    ReconstructedDistributionFromSigmaPoints(
        SigmaPointsGen const &sigmaPoints,
        TransformedSigmaPointsMat const &xformedPointsMat)
        : xformedCov_(CovMat::Zero()), crossCov_(CrossCovMatrix::Zero()) {
//! weighted average
#if 1
        xformedMean_ = MeanVec::Zero();
        for (std::size_t i = 0; i < NumSigmaPoints; ++i) {
            auto weight = sigmaPoints.getWeightsForMean()[i];
            xformedMean_ += weight * xformedPointsMat.col(i);
        }
#else
        xformedMean_ =
            xformedPointsMat.rowwise() * sigmaPoints.getWeightsForMean();
#endif
        TransformedSigmaPointsMat zeroMeanPoints =
            xformedPointsMat.colwise() - xformedMean_;

        for (std::size_t i = 0; i < NumSigmaPoints; ++i) {
            auto weight = sigmaPoints.getWeightsForCov()[i];
            xformedCov_ += weight * zeroMeanPoints.col(i) *
                           zeroMeanPoints.col(i).transpose();
            crossCov_ +=
                weight *
                (sigmaPoints.getSigmaPoint(i) - sigmaPoints.getOrigMean()) *
                zeroMeanPoints.col(i).transpose();
        }
    }

    MeanVec const &getMean() const { return xformedMean_; }
    CovMat const &getCov() const { return xformedCov_; }
    // matrix of cross-covariance between original and transformed (such as
    // state and measurement residuals)
    CrossCovMatrix const &getCrossCov() const { return crossCov_; }

  private:
    MeanVec xformedMean_;
    CovMat xformedCov_;
    CrossCovMatrix crossCov_;
};


template <typename Derived> class StateBase;
template <typename Derived> class MeasurementBase;
template <typename Derived> class ProcessModelBase;

/*!
 * The UKF parallel to CorrectionInProgress as used in an EKF.
 *
 * Initialization is done by beginUnscentedCorrection (like
 * beginExtendedCorrection). stateCorrectionFinite is provided immediately,
 * while the finishCorrection() method takes an optional bool (true by default)
 * to optionally cancel if the new error covariance is not finite.
 */
template <typename State, typename Measurement>
class SigmaPointCorrectionApplication {
  public:
    static constexpr size_t n = getDimension<State>();
    static constexpr size_t m = getDimension<Measurement>();

    using StateVec = types::Vector<n>;
    using StateSquareMatrix = types::SquareMatrix<n>;
    using MeasurementVec = types::Vector<m>;
    using MeasurementSquareMatrix = types::SquareMatrix<m>;

    //! state augmented with measurement noise mean
    static constexpr size_t AugmentedStateDim = n + m;
    using AugmentedStateVec = types::Vector<AugmentedStateDim>;
    using AugmentedStateCovMatrix = types::SquareMatrix<AugmentedStateDim>;
    using SigmaPointsGen = AugmentedSigmaPointGenerator<AugmentedStateDim, n>;

    static constexpr size_t NumSigmaPoints = SigmaPointsGen::NumSigmaPoints;

    using Reconstruction =
        ReconstructedDistributionFromSigmaPoints<m, SigmaPointsGen>;
    using TransformedSigmaPointsMat =
        typename Reconstruction::TransformedSigmaPointsMat;

    using GainMatrix = types::Matrix<n, m>;

    SigmaPointCorrectionApplication(
        State &s, Measurement &meas,
        SigmaPointParameters const &params = SigmaPointParameters())
        : state(s), measurement(meas),
          sigmaPoints(getAugmentedStateVec(s),
                      getAugmentedStateCov(s, measurement), params),
          transformedPoints(
              transformSigmaPoints(state, measurement, sigmaPoints)),
          reconstruction(sigmaPoints, transformedPoints),
          innovationCovariance(
              computeInnovationCovariance(state, measurement, reconstruction)),
          PvvDecomp(innovationCovariance.ldlt()),
          deltaz(measurement.getResidual(reconstruction.getMean(), state)),
          stateCorrection(
              computeStateCorrection(reconstruction, deltaz, PvvDecomp)),
          stateCorrectionFinite(stateCorrection.array().allFinite()) {}

    static AugmentedStateVec getAugmentedStateVec(State const &s) {
        AugmentedStateVec ret;
        //! assuming measurement noise is zero mean
        ret << s.stateVector(), MeasurementVec::Zero();
        return ret;
    }

    static AugmentedStateCovMatrix getAugmentedStateCov(State const &s,
                                                        Measurement &meas) {
        AugmentedStateCovMatrix ret;
        ret << s.errorCovariance(), types::Matrix<n, m>::Zero(),
            types::Matrix<m, n>::Zero(), meas.getCovariance(s);
        return ret;
    }

    /*!
     * Transforms sigma points by having the measurement class compute the
     * estimated measurement for a state whose state vector we update to
     * each of the sigma points in turn.
     */
    static TransformedSigmaPointsMat
    transformSigmaPoints(State const &s, Measurement &meas,
                         SigmaPointsGen const &sigmaPoints) {
        TransformedSigmaPointsMat ret;
        State tempS = s;
        for (std::size_t i = 0; i < NumSigmaPoints; ++i) {
            tempS.setStateVector(sigmaPoints.getSigmaPoint(i));
            ret.col(i) = meas.predictMeasurement(tempS);
        }
        return ret;
    }

    static MeasurementSquareMatrix
    computeInnovationCovariance(State const &s, Measurement &meas,
                                Reconstruction const &recon) {
        return recon.getCov() + meas.getCovariance(s);
    }

#if 0
    // Solve for K in K=Pxy Pvv^-1
    // where the cross-covariance matrix from the reconstruction is
    // transpose(Pxy) and Pvv is the reconstructed covariance plus the
    // measurement covariance
    static GainMatrix computeKalmanGain(MeasurementSquareMatrix const &Pvv,
                                        Reconstruction const &recon) {
        // (Actually solves with transpose(Pvv) * transpose(K) =
        // transpose(Pxy) )
        GainMatrix ret = Pvv.transpose().ldlt().solve(recon.getCrossCov());
        return ret;
    }
#endif
    static StateVec computeStateCorrection(
        Reconstruction const &recon, MeasurementVec const &deltaz,
        Eigen::LDLT<MeasurementSquareMatrix> const &pvvDecomp) {
        StateVec ret = recon.getCrossCov() * pvvDecomp.solve(deltaz);
        return ret;
    }

    /*!
     * Finish computing the rest and correct the state.
     *
     * @param cancelIfNotFinite If the new error covariance is detected to
     * contain non-finite values, should we cancel the correction and not
     * apply it?
     *
     * @return true if correction completed
     */
    bool finishCorrection(bool cancelIfNotFinite = true) {
        /*!
         * Logically state.errorCovariance() - K * Pvv * K.transpose(),
         * but considering just the second term, we can
         * replace K with its definition (Pxv Pvv^-1), distribute the
         * transpose on the right over the product, then pull out
         * Pvv^-1 * Pvv * (Pvv^-1).transpose()
         * as "B", leaving Pxv B Pxv.transpose()
         *
         * Since innovationCovariance aka Pvv is symmetric,
         * (Pvv^-1).transpose() = Pvv^-1.
         * Left multiplication gives
         * Pvv B = Pvv * Pvv^-1 * Pvv * Pvv^-1
         * whose right hand side is the Pvv-sized identity, and that is in
         * a form that allows us to use our existing LDLT decomp of Pvv to
         * solve for B then evaluate the full original expression.
         */
        StateSquareMatrix newP =
            state.errorCovariance() -
            reconstruction.getCrossCov() *
                PvvDecomp.solve(MeasurementSquareMatrix::Identity()) *
                reconstruction.getCrossCov().transpose();
        bool finite = newP.array().allFinite();
        if (cancelIfNotFinite && !finite) {
            return false;
        }

        state.setStateVector(state.stateVector() + stateCorrection);

        state.setErrorCovariance(newP);
        // Let the state do any cleanup it has to (like fixing externalized
        // quaternions)
        state.postCorrect();
        return finite;
    }

    State &state;
    Measurement &measurement;
    SigmaPointsGen sigmaPoints;
    TransformedSigmaPointsMat transformedPoints;
    Reconstruction reconstruction;
    //! aka Pvv
    MeasurementSquareMatrix innovationCovariance;
    Eigen::LDLT<MeasurementSquareMatrix> PvvDecomp;
#if 0
    GainMatrix K;
#endif
    //! reconstructed mean measurement residual/delta z/innovation
    types::Vector<m> deltaz;
    StateVec stateCorrection;
    bool stateCorrectionFinite;
};

template <typename State, typename Measurement>
inline SigmaPointCorrectionApplication<State, Measurement>
beginUnscentedCorrection(
    StateBase<State> &state, MeasurementBase<Measurement> &meas,
    SigmaPointParameters const &params = SigmaPointParameters()) {
    return SigmaPointCorrectionApplication<State, Measurement>(
        state.derived(), meas.derived(), params);
}

/*!
 * Correct a Kalman filter's state using a measurement that provides a
 * two-parameter getResidual function, in the manner of an
 * Unscented Kalman Filter (UKF).
 *
 * @param cancelIfNotFinite If the state correction or new error covariance
 * is detected to contain non-finite values, should we cancel the
 * correction and not apply it?
 *
 * @return true if correction completed
 */
template <typename State, typename Measurement>
static inline bool
correctUnscented(StateBase<State> &state, MeasurementBase<Measurement> &meas,
                 bool cancelIfNotFinite = true,
                 SigmaPointParameters const &params = SigmaPointParameters()) {

    auto inProgress = beginUnscentedCorrection(state, meas, params);
    if (cancelIfNotFinite && !inProgress.stateCorrectionFinite) {
        return false;
    }

    return inProgress.finishCorrection(cancelIfNotFinite);
}

/*!
 * A very simple (3D by default) vector state with no velocity, ideal for
 * use as a position, with ConstantProcess for beacon autocalibration
 */
template <size_t Dim = 3>
class PureVectorState : public StateBase<PureVectorState<Dim>> {
  public:
    static constexpr size_t Dimension = Dim;
    using SquareMatrix = types::SquareMatrix<Dimension>;
    using StateVector = types::Vector<Dimension>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PureVectorState(double x, double y, double z)
        : m_state(x, y, z), m_errorCovariance(SquareMatrix::Zero()) {
        static_assert(Dimension == 3, "This constructor, which takes 3 "
                                      "scalars, only works with a 3D "
                                      "vector!");
    }

    PureVectorState(double x, double y, double z,
                    SquareMatrix const &covariance)
        : m_state(x, y, z), m_errorCovariance(covariance) {
        static_assert(Dimension == 3, "This constructor, which takes 3 "
                                      "scalars, only works with a 3D "
                                      "vector!");
    }

    PureVectorState(StateVector const &state, SquareMatrix const &covariance)
        : m_state(state), m_errorCovariance(covariance) {}
    //! @name Methods required of State types
    /// @{
    //! set xhat
    void setStateVector(StateVector const &state) { m_state = state; }
    //! xhat
    StateVector const &stateVector() const { return m_state; }
    // set P
    void setErrorCovariance(SquareMatrix const &errorCovariance) {
        m_errorCovariance = errorCovariance;
    }
    //! P
    SquareMatrix const &errorCovariance() const { return m_errorCovariance; }
    void postCorrect() {}
    //! @}
  private:
    //! x
    StateVector m_state;
    //! P
    SquareMatrix m_errorCovariance;
};


/*!
 * Produces the "hat matrix" that produces the same result as
 * performing a cross-product with v. This is the same as the "capital
 * omega" skew-symmetrix matrix used by a matrix-exponential-map
 * rotation vector.
 * @param v a 3D vector
 * @return a matrix M such that for some 3D vector u, Mu = v x u.
 */
template <typename Derived>
inline Eigen::Matrix3d
makeSkewSymmetrixCrossProductMatrix(Eigen::MatrixBase<Derived> const &v) {
    Eigen::Matrix3d ret;
    // clang-format off
                ret << 0, -v.z(), v.y(),
                       v.z(), 0, -v.x(),
                       -v.y(), v.x(), 0;
    // clang-format on
    return ret;
}

/*!
 * Utilities for interacting with a "matrix exponential map vector"
 * rotation parameterization/formalism, where rotation is represented as a
 * 3D vector that is turned into a rotation matrix by applying Rodrigues'
 * formula that resembles a matrix exponential.
 *
 * Based on discussion in section 2.2.3 of:
 *
 * Lepetit, V., & Fua, P. (2005). Monocular Model-Based 3D Tracking of
 * Rigid Objects. Foundations and Trends® in Computer Graphics and Vision,
 * 1(1), 1–89. http://doi.org/10.1561/0600000001
 *
 * Not to be confused with the quaternion-related exponential map espoused
 * in:
 *
 * Grassia, F. S. (1998). Practical Parameterization of Rotations Using the
 * Exponential Map. Journal of Graphics Tools, 3(3), 29–48.
 * http://doi.org/10.1080/10867651.1998.10487493
 */
namespace matrix_exponential_map {
    /*!
     * Adjust a matrix exponential map rotation vector, if required, to
     * avoid  singularities.
     * @param omega a 3D "matrix exponential map" rotation vector, which
     * will be modified if required.
     */
    template <typename T> inline void avoidSingularities(T &&omega) {
        // if magnitude gets too close to 2pi, in this case, pi...
        if (omega.squaredNorm() > EIGEN_PI * EIGEN_PI) {
            // replace omega with an equivalent one.
            omega = ((1 - (2 * EIGEN_PI) / omega.norm()) * omega).eval();
        }
    }

    /*!
     * Gets the rotation angle of a rotation vector.
     * @param omega a 3D "exponential map" rotation vector
     */
    template <typename Derived>
    inline double getAngle(Eigen::MatrixBase<Derived> const &omega) {
        return omega.norm();
    }

    /*!
     * Gets the unit quaternion corresponding to the exponential rotation
     * vector.
     * @param omega a 3D "exponential map" rotation vector
     */
    template <typename Derived>
    inline Eigen::Quaterniond getQuat(Eigen::MatrixBase<Derived> const &omega) {
        auto theta = getAngle(omega);
        auto xyz = omega * std::sin(theta / 2.);
        return Eigen::Quaterniond(std::cos(theta / 2.), xyz.x(), xyz.y(),
                                  xyz.z());
    }

    //! Contained cached computed values
    class ExponentialMapData {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*!
         * Construct from a matrixy-thing: should be a 3d vector containing
         * a matrix-exponential-map rotation formalism.
         */
        template <typename Derived>
        explicit ExponentialMapData(Eigen::MatrixBase<Derived> const &omega)
            : m_omega(omega) {}

        ExponentialMapData() : m_omega(Eigen::Vector3d::Zero()) {}

        //! assignment operator - its presence is an optimization only.
        ExponentialMapData &operator=(ExponentialMapData const &other) {
            if (&other != this) {
                m_omega = other.m_omega;
                m_gotTheta = other.m_gotTheta;
                if (m_gotTheta) {
                    m_theta = other.m_theta;
                }
                m_gotBigOmega = other.m_gotBigOmega;
                if (m_gotBigOmega) {
                    m_bigOmega = other.m_bigOmega;
                }
                m_gotR = other.m_gotR;
                if (m_gotR) {
                    m_R = other.m_R;
                }
                m_gotQuat = other.m_gotQuat;
                if (m_gotQuat) {
                    m_quat = other.m_quat;
                }
            }
            return *this;
        }

        //! move-assignment operator - its presence is an optimization only.
        ExponentialMapData &operator=(ExponentialMapData &&other) {
            if (&other != this) {
                m_omega = std::move(other.m_omega);
                m_gotTheta = std::move(other.m_gotTheta);
                if (m_gotTheta) {
                    m_theta = std::move(other.m_theta);
                }
                m_gotBigOmega = std::move(other.m_gotBigOmega);
                if (m_gotBigOmega) {
                    m_bigOmega = std::move(other.m_bigOmega);
                }
                m_gotR = std::move(other.m_gotR);
                if (m_gotR) {
                    m_R = std::move(other.m_R);
                }
                m_gotQuat = std::move(other.m_gotQuat);
                if (m_gotQuat) {
                    m_quat = std::move(other.m_quat);
                }
            }
            return *this;
        }

        template <typename Derived>
        void reset(Eigen::MatrixBase<Derived> const &omega) {
            //! Using assignment operator to be sure I didn't miss a flag.
            *this = ExponentialMapData(omega);
        }

        /*!
         * Gets the "capital omega" skew-symmetrix matrix.
         *
         * (computation is cached)
         */
        Eigen::Matrix3d const &getBigOmega() {
            if (!m_gotBigOmega) {
                m_gotBigOmega = true;
                m_bigOmega = makeSkewSymmetrixCrossProductMatrix(m_omega);
            }
            return m_bigOmega;
        }

        /*!
         * Gets the rotation angle of a rotation vector.
         *
         * (computation is cached)
         */
        double getTheta() {
            if (!m_gotTheta) {
                m_gotTheta = true;
                m_theta = getAngle(m_omega);
            }
            return m_theta;
        }

        /*!
         * Converts a rotation vector to a rotation matrix:
         * Uses Rodrigues' formula, and the first two terms of the Taylor
         * expansions of the trig functions (so as to be nonsingular as the
         * angle goes to zero).
         *
         * (computation is cached)
         */
        Eigen::Matrix3d const &getRotationMatrix() {
            if (!m_gotR) {
                m_gotR = true;
                auto theta = getTheta();
                auto &Omega = getBigOmega();
                //! two-term taylor approx of sin(theta)/theta
                double k1 = 1. - theta * theta / 6.;

                //! two-term taylor approx of (1-cos(theta))/theta
                double k2 = theta / 2. - theta * theta * theta / 24.;

                m_R = Eigen::Matrix3d::Identity() + k1 * Omega +
                      k2 * Omega * Omega;
            }
            return m_R;
        }

        Eigen::Quaterniond const &getQuaternion() {
            if (!m_gotQuat) {
                m_gotQuat = true;
                auto theta = getTheta();
                auto xyz = m_omega * std::sin(theta / 2.);
                m_quat = Eigen::Quaterniond(std::cos(theta / 2.), xyz.x(),
                                            xyz.y(), xyz.z());
            }
            return m_quat;
        }

      private:
        Eigen::Vector3d m_omega;
        bool m_gotTheta = false;
        double m_theta;
        bool m_gotBigOmega = false;
        Eigen::Matrix3d m_bigOmega;
        bool m_gotR = false;
        Eigen::Matrix3d m_R;
        bool m_gotQuat = false;
        Eigen::Quaterniond m_quat;
    };

} // namespace matrix_exponential_map


namespace pose_exp_map {

    constexpr size_t Dimension = 12;
    using StateVector = types::Vector<Dimension>;
    using StateVectorBlock3 = StateVector::FixedSegmentReturnType<3>::Type;
    using ConstStateVectorBlock3 =
        StateVector::ConstFixedSegmentReturnType<3>::Type;

    using StateVectorBlock6 = StateVector::FixedSegmentReturnType<6>::Type;
    using StateSquareMatrix = types::SquareMatrix<Dimension>;

    //! @name Accessors to blocks in the state vector.
    /// @{
    inline StateVectorBlock3 position(StateVector &vec) {
        return vec.head<3>();
    }
    inline ConstStateVectorBlock3 position(StateVector const &vec) {
        return vec.head<3>();
    }

    inline StateVectorBlock3 orientation(StateVector &vec) {
        return vec.segment<3>(3);
    }
    inline ConstStateVectorBlock3 orientation(StateVector const &vec) {
        return vec.segment<3>(3);
    }

    inline StateVectorBlock3 velocity(StateVector &vec) {
        return vec.segment<3>(6);
    }
    inline ConstStateVectorBlock3 velocity(StateVector const &vec) {
        return vec.segment<3>(6);
    }

    inline StateVectorBlock3 angularVelocity(StateVector &vec) {
        return vec.segment<3>(9);
    }
    inline ConstStateVectorBlock3 angularVelocity(StateVector const &vec) {
        return vec.segment<3>(9);
    }

    //! both translational and angular velocities
    inline StateVectorBlock6 velocities(StateVector &vec) {
        return vec.segment<6>(6);
    }
    //! @}

    /*!
     * This returns A(deltaT), though if you're just predicting xhat-, use
     * applyVelocity() instead for performance.
     */
    inline StateSquareMatrix stateTransitionMatrix(double dt) {
        // eq. 4.5 in Welch 1996 - except we have all the velocities at the
        // end
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<6, 6>() = types::SquareMatrix<6>::Identity() * dt;

        return A;
    }
    inline double computeAttenuation(double damping, double dt) {
        return std::pow(damping, dt);
    }
    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(double dt, double damping) {

        // eq. 4.5 in Welch 1996

        auto A = stateTransitionMatrix(dt);
        auto attenuation = computeAttenuation(damping, dt);
        A.bottomRightCorner<6, 6>() *= attenuation;
        return A;
    }
    //! Computes A(deltaT)xhat(t-deltaT)
    inline StateVector applyVelocity(StateVector const &state, double dt) {
        // eq. 4.5 in Welch 1996

        /*!
         * @todo benchmark - assuming for now that the manual small
         * calcuations are faster than the matrix ones.
         */

        StateVector ret = state;
        position(ret) += velocity(state) * dt;
        orientation(ret) += angularVelocity(state) * dt;
        return ret;
    }

    inline void dampenVelocities(StateVector &state, double damping,
                                 double dt) {
        auto attenuation = computeAttenuation(damping, dt);
        velocities(state) *= attenuation;
    }
    class State : public StateBase<State> {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        static constexpr size_t Dimension = 12;

        //! Default constructor
        State()
            : m_state(StateVector::Zero()),
              m_errorCovariance(
                  StateSquareMatrix::
                      Identity() /** @todo almost certainly wrong */) {}
        //! set xhat
        void setStateVector(StateVector const &state) { m_state = state; }
        //! xhat
        StateVector const &stateVector() const { return m_state; }
        // set P
        void setErrorCovariance(StateSquareMatrix const &errorCovariance) {
            m_errorCovariance = errorCovariance;
        }
        //! P
        StateSquareMatrix const &errorCovariance() const {
            return m_errorCovariance;
        }

        void postCorrect() {
            matrix_exponential_map::avoidSingularities(orientation(m_state));
            m_cacheData.reset(Eigen::Vector3d(orientation(m_state)));
        }

        StateVectorBlock3 position() { return pose_exp_map::position(m_state); }

        ConstStateVectorBlock3 position() const {
            return pose_exp_map::position(m_state);
        }

        Eigen::Quaterniond const &getQuaternion() {
            return m_cacheData.getQuaternion();
        }
        Eigen::Matrix3d const &getRotationMatrix() {
            return m_cacheData.getRotationMatrix();
        }

        StateVectorBlock3 velocity() { return pose_exp_map::velocity(m_state); }

        ConstStateVectorBlock3 velocity() const {
            return pose_exp_map::velocity(m_state);
        }

        StateVectorBlock3 angularVelocity() {
            return pose_exp_map::angularVelocity(m_state);
        }

        ConstStateVectorBlock3 angularVelocity() const {
            return pose_exp_map::angularVelocity(m_state);
        }

      private:
        /*!
         * In order: x, y, z, exponential rotation coordinates w1, w2, w3,
         * then their derivatives in the same order.
         */
        StateVector m_state;
        //! P
        StateSquareMatrix m_errorCovariance;

        //! Cached data for use in consuming the exponential map rotation.
        matrix_exponential_map::ExponentialMapData m_cacheData;
    };

    /*!
     * Stream insertion operator, for displaying the state of the state
     * class.
     */
    template <typename OutputStream>
    inline OutputStream &operator<<(OutputStream &os, State const &state) {
        os << "State:" << state.stateVector().transpose() << "\n";
        os << "error:\n" << state.errorCovariance() << "\n";
        return os;
    }
} // namespace pose_exp_map


/*!
 * State type that consists entirely of references to two independent
 * sub-states.
 */
template <typename StateA, typename StateB>
class AugmentedState : public StateBase<AugmentedState<StateA, StateB>> {
  public:
    using StateTypeA = StateA;
    using StateTypeB = StateB;

    static constexpr size_t DimA = getDimension<StateA>();
    static constexpr size_t DimB = getDimension<StateB>();
    static constexpr size_t Dimension = DimA + DimB;

    using SquareMatrix = types::SquareMatrix<Dimension>;
    using StateVector = types::Vector<Dimension>;

    //! Constructor
    AugmentedState(StateA &a, StateB &b) : a_(std::ref(a)), b_(std::ref(b)) {}

    //! @name Methods required of State types
    /// @{
    template <typename Derived>
    void setStateVector(Eigen::MatrixBase<Derived> const &state) {
        //! template used here to avoid a temporary
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, Dimension);
        a().setStateVector(state.derived().template head<DimA>());
        b().setStateVector(state.derived().template tail<DimB>());
    }

    StateVector stateVector() const {
        StateVector ret;
        ret << a().stateVector(), b().stateVector();
        return ret;
    }

    SquareMatrix errorCovariance() const {
        SquareMatrix ret = SquareMatrix::Zero();
        ret.template topLeftCorner<DimA, DimA>() = a().errorCovariance();
        ret.template bottomRightCorner<DimB, DimB>() = b().errorCovariance();
        return ret;
    }

    template <typename Derived>
    void setErrorCovariance(Eigen::MatrixBase<Derived> const &P) {
        /*!
         * template used here to avoid evaluating elements we'll never
         * access to a temporary.
         */
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, Dimension, Dimension);
        a().setErrorCovariance(P.template topLeftCorner<DimA, DimA>());
        b().setErrorCovariance(P.template bottomRightCorner<DimB, DimB>());
    }

    void postCorrect() {
        a().postCorrect();
        b().postCorrect();
    }
    //! @}

    //! @name Access to the components of the state
    /// @{
    //! Access the first part of the state
    StateTypeA &a() { return a_.get(); }
    //! Access the first part of the state
    StateTypeA const &a() const { return a_.get(); }

    //! Access the second part of the state
    StateTypeB &b() { return b_.get(); }
    //! Access the second part of the state
    StateTypeB const &b() const { return b_.get(); }
    //! @}

  private:
    std::reference_wrapper<StateA> a_;
    std::reference_wrapper<StateB> b_;
};

/*!
 * Template alias to make removing const from the deduced types less
 * verbose/painful.
 */
template <typename StateA, typename StateB>
using DeducedAugmentedState =
    AugmentedState<typename std::remove_const<StateA>::type,
                   typename std::remove_const<StateB>::type>;

//! Factory function, akin to `std::tie()`, to make an augmented state.
template <typename StateA, typename StateB>
inline DeducedAugmentedState<StateA, StateB> makeAugmentedState(StateA &a,
                                                                StateB &b) {
    return {a, b};
}


/*!
 * Process model type that consists entirely of references to two
 * sub-process models, for operating on an AugmentedState<>.
 */
template <typename ModelA, typename ModelB>
class AugmentedProcessModel
    : public ProcessModelBase<AugmentedProcessModel<ModelA, ModelB>> {
  public:
    using ModelTypeA = ModelA;
    using ModelTypeB = ModelB;
    using StateA = typename ModelA::State;
    using StateB = typename ModelB::State;
    using State = AugmentedState<StateA, StateB>;

    //! Constructor
    AugmentedProcessModel(ModelTypeA &modA, ModelTypeB &modB)
        : a_(std::ref(modA)), b_(std::ref(modB)) {}

    //! @name Method required of Process Model types
    /// @{
    void predictState(State &state, double dt) {
        modelA().predictState(state.a(), dt);
        modelB().predictState(state.b(), dt);
    }
    //! @}

    //! @name Access to the components of the process model
    /// @{
    ModelTypeA &modelA() { return a_.get(); }
    ModelTypeA const &modelA() const { return a_.get(); }

    ModelTypeB &modelB() { return b_.get(); }
    ModelTypeB const &modelB() const { return b_.get(); }
    //! @}
  private:
    std::reference_wrapper<ModelTypeA> a_;
    std::reference_wrapper<ModelTypeB> b_;
};

/*!
 * Template alias to make removing const from the deduced types less
 * verbose/painful.
 */
template <typename ModelA, typename ModelB>
using DeducedAugmentedProcessModel =
    AugmentedProcessModel<typename std::remove_const<ModelA>::type,
                          typename std::remove_const<ModelB>::type>;

/*!
 * Factory function, akin to `std::tie()`, to make an augmented process
 * model.
 */
template <typename ModelA, typename ModelB>
inline DeducedAugmentedProcessModel<ModelA, ModelB>
makeAugmentedProcessModel(ModelA &a, ModelB &b) {
    return {a, b};
}


namespace orient_externalized_rotation {
    constexpr size_t Dimension = 6;
    using StateVector = types::Vector<Dimension>;
    using StateVectorBlock3 = StateVector::FixedSegmentReturnType<3>::Type;
    using ConstStateVectorBlock3 =
        StateVector::ConstFixedSegmentReturnType<3>::Type;

    using StateSquareMatrix = types::SquareMatrix<Dimension>;

    /*!
     * This returns A(deltaT), though if you're just predicting xhat-, use
     * applyVelocity() instead for performance.
     */
    inline StateSquareMatrix stateTransitionMatrix(double dt) {
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity() * dt;
        return A;
    }
    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(double dt, double damping) {

        // eq. 4.5 in Welch 1996

        auto A = stateTransitionMatrix(dt);
        auto attenuation = std::pow(damping, dt);
        A.bottomRightCorner<3, 3>() *= attenuation;
        return A;
    }
    class State : public StateBase<State> {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static constexpr size_t Dimension = 6;

        //! Default constructor
        State()
            : m_state(StateVector::Zero()),
              m_errorCovariance(
                  StateSquareMatrix::
                      Identity() /** @todo almost certainly wrong */),
              m_orientation(Eigen::Quaterniond::Identity()) {}
        //! set xhat
        void setStateVector(StateVector const &state) { m_state = state; }
        //! xhat
        StateVector const &stateVector() const { return m_state; }
        // set P
        void setErrorCovariance(StateSquareMatrix const &errorCovariance) {
            m_errorCovariance = errorCovariance;
        }
        //! P
        StateSquareMatrix const &errorCovariance() const {
            return m_errorCovariance;
        }

        //! Intended for startup use.
        void setQuaternion(Eigen::Quaterniond const &quaternion) {
            m_orientation = quaternion.normalized();
        }

        void postCorrect() { externalizeRotation(); }

        void externalizeRotation() {
            m_orientation = getCombinedQuaternion();
            incrementalOrientation() = Eigen::Vector3d::Zero();
        }

        void normalizeQuaternion() { m_orientation.normalize(); }

        StateVectorBlock3 incrementalOrientation() { return m_state.head<3>(); }

        ConstStateVectorBlock3 incrementalOrientation() const {
            return m_state.head<3>();
        }

        StateVectorBlock3 angularVelocity() { return m_state.tail<3>(); }

        ConstStateVectorBlock3 angularVelocity() const {
            return m_state.tail<3>();
        }

        Eigen::Quaterniond const &getQuaternion() const {
            return m_orientation;
        }

        Eigen::Quaterniond getCombinedQuaternion() const {
            // divide by 2 since we're integrating it essentially.
            return util::quat_exp(incrementalOrientation() / 2.) *
                   m_orientation;
        }

      private:
        /*!
         * In order: x, y, z, orientation , then its derivatives in the
         * same
         * order.
         */
        StateVector m_state;
        //! P
        StateSquareMatrix m_errorCovariance;
        //! Externally-maintained orientation per Welch 1996
        Eigen::Quaterniond m_orientation;
    };

    /*!
     * Stream insertion operator, for displaying the state of the state
     * class.
     */
    template <typename OutputStream>
    inline OutputStream &operator<<(OutputStream &os, State const &state) {
        os << "State:" << state.stateVector().transpose() << "\n";
        os << "quat:" << state.getCombinedQuaternion().coeffs().transpose()
           << "\n";
        os << "error:\n" << state.errorCovariance() << "\n";
        return os;
    }

    //! Computes A(deltaT)xhat(t-deltaT)
    inline void applyVelocity(State &state, double dt) {
        // eq. 4.5 in Welch 1996

        /*!
         * @todo benchmark - assuming for now that the manual small
         * calcuations are faster than the matrix ones.
         */

        state.incrementalOrientation() += state.angularVelocity() * dt;
    }

    inline void dampenVelocities(State &state, double damping, double dt) {
        auto attenuation = std::pow(damping, dt);
        state.angularVelocity() *= attenuation;
    }

} // namespace orient_externalized_rotation


//! A model for a 3DOF pose (with angular velocity)
class OrientationConstantVelocityProcessModel
    : public ProcessModelBase<OrientationConstantVelocityProcessModel> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = orient_externalized_rotation::State;
    using StateVector = orient_externalized_rotation::StateVector;
    using StateSquareMatrix = orient_externalized_rotation::StateSquareMatrix;
    using NoiseAutocorrelation = types::Vector<3>;
    OrientationConstantVelocityProcessModel(double orientationNoise = 0.1) {
        setNoiseAutocorrelation(orientationNoise);
    }
    void setNoiseAutocorrelation(double orientationNoise = 0.1) {
        m_mu.head<3>() = types::Vector<3>::Constant(orientationNoise);
    }
    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_mu = noise;
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &, double dt) const {
        return orient_externalized_rotation::stateTransitionMatrix(dt);
    }

    void predictStateOnly(State &s, double dt) const {
        FLEXKALMAN_DEBUG_OUTPUT("Time change", dt);
        orient_externalized_rotation::applyVelocity(s, dt);
    }
    void predictState(State &s, double dt) const {
        predictStateOnly(s, dt);
        auto Pminus = predictErrorCovariance(s, *this, dt);
        s.setErrorCovariance(Pminus);
    }

    /*!
     * This is Q(deltaT) - the Sampled Process Noise Covariance
     * @return a matrix of dimension n x n.
     *
     * Like all covariance matrices, it is real symmetrical (self-adjoint),
     * so .selfAdjointView<Eigen::Upper>() might provide useful performance
     * enhancements in some algorithms.
     */
    StateSquareMatrix getSampledProcessNoiseCovariance(double dt) const {
        constexpr auto dim = getDimension<State>();
        StateSquareMatrix cov = StateSquareMatrix::Zero();
        auto dt3 = (dt * dt * dt) / 3;
        auto dt2 = (dt * dt) / 2;
        for (std::size_t xIndex = 0; xIndex < dim / 2; ++xIndex) {
            auto xDotIndex = xIndex + dim / 2;
            // xIndex is 'i' and xDotIndex is 'j' in eq. 4.8
            const auto mu = getMu(xDotIndex);
            cov(xIndex, xIndex) = mu * dt3;
            auto symmetric = mu * dt2;
            cov(xIndex, xDotIndex) = symmetric;
            cov(xDotIndex, xIndex) = symmetric;
            cov(xDotIndex, xDotIndex) = mu * dt;
        }
        return cov;
    }

  private:
    /*!
     * this is mu-arrow, the auto-correlation vector of the noise
     * sources
     */
    NoiseAutocorrelation m_mu;
    double getMu(std::size_t index) const {
        assert(index < (getDimension<State>() / 2) &&
               "Should only be passing "
               "'i' - the main state, not "
               "the derivative");
        // This may not be totally correct but it's one of the parameters
        // you can kind of fudge in kalman filters anyway.
        // Should techincally be the diagonal of the correlation kernel of
        // the noise sources. (p77, p197 in Welch 1996)
        return m_mu(index);
    }
};


/*!
 * A basically-constant-velocity model, with the addition of some
 * damping of the velocities inspired by TAG
 */
class PoseDampedConstantVelocityProcessModel
    : public ProcessModelBase<PoseDampedConstantVelocityProcessModel> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = pose_externalized_rotation::State;
    using StateVector = pose_externalized_rotation::StateVector;
    using StateSquareMatrix = pose_externalized_rotation::StateSquareMatrix;
    using BaseProcess = PoseConstantVelocityProcessModel;
    using NoiseAutocorrelation = BaseProcess::NoiseAutocorrelation;
    PoseDampedConstantVelocityProcessModel(double damping = 0.1,
                                           double positionNoise = 0.01,
                                           double orientationNoise = 0.1)
        : m_constantVelModel(positionNoise, orientationNoise) {
        setDamping(damping);
    }

    void setNoiseAutocorrelation(double positionNoise = 0.01,
                                 double orientationNoise = 0.1) {
        m_constantVelModel.setNoiseAutocorrelation(positionNoise,
                                                   orientationNoise);
    }

    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_constantVelModel.setNoiseAutocorrelation(noise);
    }
    //! Set the damping - must be positive
    void setDamping(double damping) {
        if (damping > 0) {
            m_damp = damping;
        }
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &, double dt) const {
        return pose_externalized_rotation::
            stateTransitionMatrixWithVelocityDamping(dt, m_damp);
    }

    void predictStateOnly(State &s, double dt) const {
        m_constantVelModel.predictStateOnly(s, dt);
        // Dampen velocities
        pose_externalized_rotation::dampenVelocities(s, m_damp, dt);
    }

    void predictState(State &s, double dt) const {
        predictStateOnly(s, dt);
        auto Pminus = predictErrorCovariance(s, *this, dt);
        s.setErrorCovariance(Pminus);
    }

    /*!
     * This is Q(deltaT) - the Sampled Process Noise Covariance
     * @return a matrix of dimension n x n. Note that it is real
     * symmetrical (self-adjoint), so .selfAdjointView<Eigen::Upper>()
     * might provide useful performance enhancements.
     */
    StateSquareMatrix getSampledProcessNoiseCovariance(double dt) const {
        return m_constantVelModel.getSampledProcessNoiseCovariance(dt);
    }

  private:
    BaseProcess m_constantVelModel;
    double m_damp = 0.1;
};

class AbsolutePositionMeasurementBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const size_t Dimension = 3; // 3 position
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementDiagonalMatrix = types::DiagonalMatrix<Dimension>;
    using MeasurementMatrix = types::SquareMatrix<Dimension>;
    AbsolutePositionMeasurementBase(MeasurementVector const &pos,
                                    MeasurementVector const &variance)
        : m_pos(pos), m_covariance(variance.asDiagonal()) {}

    template <typename State>
    MeasurementMatrix getCovariance(State const &) const {
        return m_covariance;
    }

    /*!
     * Gets the measurement residual, also known as innovation: predicts
     * the measurement from the predicted state, and returns the
     * difference.
     *
     * State type doesn't matter as long as we can `.position()`
     */
    template <typename State>
    MeasurementVector getResidual(State const &s) const {
        MeasurementVector residual = m_pos - s.position();
        return residual;
    }

    //! Convenience method to be able to store and re-use measurements.
    void setMeasurement(MeasurementVector const &pos) { m_pos = pos; }

  private:
    MeasurementVector m_pos;
    MeasurementDiagonalMatrix m_covariance;
};
/*!
 * This class is a 3D position measurement.
 *
 * It can be used with any state class that exposes a `position()`
 * method. On its own, it is only suitable for unscented filter correction,
 * since the jacobian depends on the arrangement of the state vector. See
 * AbsolutePositionEKFMeasurement's explicit specializations for use in EKF
 * correction mode.
 */
class AbsolutePositionMeasurement
    : public AbsolutePositionMeasurementBase,
      public MeasurementBase<AbsolutePositionMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using AbsolutePositionMeasurementBase::AbsolutePositionMeasurementBase;
};

/*!
 * This is the EKF-specific relative of AbsolutePositionMeasurement: only
 * explicit specializations, and on state types.
 */
template <typename StateType> class AbsolutePositionEKFMeasurement;

//! AbsolutePositionEKFMeasurement with a pose_externalized_rotation::State
template <>
class AbsolutePositionEKFMeasurement<pose_externalized_rotation::State>
    : public AbsolutePositionMeasurementBase,
      public MeasurementBase<
          AbsolutePositionEKFMeasurement<pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    using AbsolutePositionMeasurementBase::Dimension;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t StateDimension = getDimension<State>();
    using Jacobian = types::Matrix<Dimension, StateDimension>;
    AbsolutePositionEKFMeasurement(MeasurementVector const &pos,
                                   MeasurementVector const &variance)
        : AbsolutePositionMeasurementBase(pos, variance),
          m_jacobian(Jacobian::Zero()) {
        m_jacobian.block<3, 3>(0, 0) = types::SquareMatrix<3>::Identity();
    }

    types::Matrix<Dimension, StateDimension> const &
    getJacobian(State const &) const {
        return m_jacobian;
    }

  private:
    types::Matrix<Dimension, StateDimension> m_jacobian;
};


/*!
 * A simple process model for a "constant" process: all prediction does at
 * most is bump up the uncertainty. Since it's widely applicable, it's
 * templated on state type.
 *
 * One potential application is for beacon autocalibration in a device
 * filter.
 */
template <typename StateType>
class ConstantProcess : public ProcessModelBase<ConstantProcess<StateType>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = StateType;
    static constexpr size_t Dimension = getDimension<State>();
    using StateVector = types::Vector<Dimension>;
    using StateSquareMatrix = types::SquareMatrix<Dimension>;
    ConstantProcess() : m_constantNoise(StateSquareMatrix::Zero()) {}
    void predictState(State &state, double dt) {

        // Predict a-priori P
        // The formula for this prediction is AP(A^T) + Q, where Q is
        // getSampledProcessNoiseCovariance, and A is
        // getStateTransitionMatrix, both optional process model methods.
        // Since the state transition matrix for this, a constant process,
        // is just the identity, this simplifies to a sum, so we just
        // directly do the computation here rather than calling the
        // predictErrorCovariance() free function.
        StateSquareMatrix Pminus =
            state.errorCovariance() + dt * m_constantNoise;
        state.setErrorCovariance(Pminus);
    }
    void setNoiseAutocorrelation(double noise) {
        m_constantNoise = StateVector::Constant(noise).asDiagonal();
    }

    void setNoiseAutocorrelation(StateVector const &noise) {
        m_constantNoise = noise.asDiagonal;
    }

  private:
    StateSquareMatrix m_constantNoise;
};


class AngularVelocityMeasurementBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    AngularVelocityMeasurementBase(MeasurementVector const &vel,
                                   MeasurementVector const &variance)
        : m_measurement(vel), m_covariance(variance.asDiagonal()) {}

    template <typename State>
    MeasurementSquareMatrix const &getCovariance(State const &) {
        return m_covariance;
    }

    template <typename State>
    types::Vector<3> predictMeasurement(State const &s) const {
        return s.angularVelocity();
    }

    template <typename State>
    MeasurementVector getResidual(MeasurementVector const &prediction,
                                  State const & /* s */) const {
        const MeasurementVector residual = m_measurement - prediction;
        return residual;
    }

    /*!
     * Gets the measurement residual, also known as innovation: predicts
     * the measurement from the predicted state, and returns the
     * difference.
     *
     * State type doesn't matter as long as we can `.angularVelocity()`
     */
    template <typename State>
    MeasurementVector getResidual(State const &s) const {
        return getResidual(predictMeasurement(s), s);
    }

    //! Convenience method to be able to store and re-use measurements.
    void setMeasurement(MeasurementVector const &vel) { m_measurement = vel; }

  private:
    MeasurementVector m_measurement;
    MeasurementSquareMatrix m_covariance;
};
/*!
 * This class is a 3D angular velocity measurement.
 *
 * It can be used with any state class that exposes a `angularVelocity()`
 * method. On its own, it is only suitable for unscented filter correction,
 * since the jacobian depends on the arrangement of the state vector. See
 * AngularVelocityEKFMeasurement's explicit specializations for use in EKF
 * correction mode.
 */
class AngularVelocityMeasurement
    : public AngularVelocityMeasurementBase,
      public MeasurementBase<AngularVelocityMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using AngularVelocityMeasurementBase::AngularVelocityMeasurementBase;
};

/*!
 * This is the EKF-specific relative of AngularVelocityMeasurement: only
 * explicit specializations, and on state types.
 *
 * Only required for EKF-style correction (since jacobian depends closely on the
 * state).
 */
template <typename StateType> class AngularVelocityEKFMeasurement;

//! AngularVelocityEKFMeasurement with a pose_externalized_rotation::State
template <>
class AngularVelocityEKFMeasurement<pose_externalized_rotation::State>
    : public AngularVelocityMeasurementBase,
      public MeasurementBase<
          AngularVelocityEKFMeasurement<pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    static constexpr size_t StateDimension = getDimension<State>();
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AngularVelocityEKFMeasurement(MeasurementVector const &vel,
                                  MeasurementVector const &variance)
        : AngularVelocityMeasurementBase(vel, variance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &) const {
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity();
        return ret;
    }
};

/*!
 * AngularVelocityEKFMeasurement with a orient_externalized_rotation::State
 * The code is in fact identical except for the state types, due to a
 * coincidence of how the state vectors are arranged.
 */
template <>
class AngularVelocityEKFMeasurement<orient_externalized_rotation::State>
    : public AngularVelocityMeasurementBase,
      public MeasurementBase<
          AngularVelocityEKFMeasurement<orient_externalized_rotation::State>> {
  public:
    using State = orient_externalized_rotation::State;
    static constexpr size_t StateDimension = getDimension<State>();
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AngularVelocityEKFMeasurement(MeasurementVector const &vel,
                                  MeasurementVector const &variance)
        : AngularVelocityMeasurementBase(vel, variance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &) const {
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity();
        return ret;
    }
};

} // namespace flexkalman
