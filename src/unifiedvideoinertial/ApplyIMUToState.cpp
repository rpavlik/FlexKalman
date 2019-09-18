/** @file
    @brief Implementation

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

// Internal Includes
#include "ApplyIMUToState.h"
#include "unifiedvideoinertial/AngVelTools.h"
#include "unifiedvideoinertial/CrossProductMatrix.h"
#include "unifiedvideoinertial/IMUStateMeasurements.h"
#include "unifiedvideoinertial/SpaceTransformations.h"

// Library/third-party includes
#include "FlexKalman/AbsoluteOrientationMeasurement.h"
#include "FlexKalman/AngularVelocityMeasurement.h"
#include "FlexKalman/EigenQuatExponentialMap.h"
#include "FlexKalman/FlexibleKalmanFilter.h"
#include "FlexKalman/FlexibleUnscentedCorrect.h"
#include "unifiedvideoinertial/EigenExtras.h"
#include "unifiedvideoinertial/Stride.h"

// Standard includes
#include <iostream>

namespace videotracker {
namespace uvbi {
    using videotracker::util::Angle;
    inline void applyOriToState(TrackingSystem const &sys, BodyState &state,
                                BodyProcessModel &processModel,
                                CannedIMUMeasurement const &meas) {
        Eigen::Quaterniond quat;
        meas.restoreQuat(quat);
        Eigen::Vector3d var;
        meas.restoreQuatVariance(var);
        Angle yawCorrection = meas.getYawCorrection();

        Eigen::Quaterniond quatInCamSpace = getTransformedOrientation(
            quat, Eigen::Quaterniond(sys.getRoomToCamera().rotation()),
            yawCorrection);
        OrientationMeasurement kalmanMeas{quatInCamSpace, var};
#if 0
        auto correctionInProgress =
            flexkalman::beginCorrection(state, processModel, kalmanMeas);
#else
        auto correctionInProgress =
            flexkalman::beginUnscentedCorrection(state, kalmanMeas);
#endif
        auto outputMeas = [&] {
            std::cout << "state: " << state.getQuaternion().coeffs().transpose()
                      << " and measurement: "
#if 0
                      << quat.coeffs().transpose();
#else
                      << quatInCamSpace.coeffs().transpose();
#endif
        };
        if (!correctionInProgress.stateCorrectionFinite) {
            std::cout
                << "Non-finite state correction in applying orientation: ";
            outputMeas();
            std::cout << "\n";
            return;
        }
#if 0
        static ::util::Stride s(401);
        if (++s) {

            std::cout << "delta z\t "
                      << correctionInProgress.deltaz.transpose();
            std::cout << "\t state correction "
                      << correctionInProgress.stateCorrection.transpose()
                      << "\n";
        }
#endif
        if (!correctionInProgress.finishCorrection(true)) {
            std::cout
                << "Non-finite error covariance after applying orientation: ";
            outputMeas();
            std::cout << "\n";
        }
    }

    inline void applyAngVelToState(TrackingSystem const &sys, BodyState &state,
                                   BodyProcessModel &processModel,
                                   CannedIMUMeasurement const &meas) {

        Eigen::Vector3d angVel;
        meas.restoreAngVel(angVel);
        Eigen::Vector3d var;
        meas.restoreAngVelVariance(var);
#if 0
        static const double dt = 0.02;
        /// Rotate it into camera space - it's bTb and we want cTc
        /// @todo do this without rotating into camera space?
        Eigen::Quaterniond cTb = state.getQuaternion();
        // Eigen::Matrix3d bTc(state.getQuaternion().conjugate());
        /// Turn it into an incremental quat to do the space transformation
        Eigen::Quaterniond incrementalQuat =
            cTb * angVelVecToIncRot(angVel, dt) * cTb.conjugate();
        /// Then turn it back into an angular velocity vector
        angVel = incRotToAngVelVec(incrementalQuat, dt);
        // angVel = (getRotationMatrixToCameraSpace(sys) * angVel).eval();
        /// @todo transform variance?

        flexkalman::AngularVelocityEKFMeasurement<BodyState> kalmanMeas{angVel, var};
        flexkalman::correct(state, processModel, kalmanMeas);
#else
        flexkalman::IMUAngVelMeasurement kalmanMeas{angVel, var};
        auto correctionInProgress =
            flexkalman::beginUnscentedCorrection(state, kalmanMeas);
        auto outputMeas = [&] {
            std::cout << "state: " << state.angularVelocity().transpose()
                      << " and measurement: " << angVel.transpose();
        };
        if (!correctionInProgress.stateCorrectionFinite) {
            std::cout
                << "Non-finite state correction in applying angular velocity: ";
            outputMeas();
            std::cout << "\n";
            return;
        }
        if (!correctionInProgress.finishCorrection(true)) {
            std::cout << "Non-finite error covariance after applying angular "
                         "velocity: ";
            outputMeas();
            std::cout << "\n";
        }
#endif
    }

    void applyIMUToState(TrackingSystem const &sys,
                         util::TimeValue const &initialTime, BodyState &state,
                         BodyProcessModel &processModel,
                         util::TimeValue const &newTime,
                         CannedIMUMeasurement const &meas) {
        if (newTime != initialTime) {
            auto dt = uvbiTimeValueDurationSeconds(&newTime, &initialTime);
            flexkalman::predict(state, processModel, dt);
#if 0
            state.externalizeRotation();
#endif
        }
        if (meas.orientationValid()) {
            applyOriToState(sys, state, processModel, meas);
        } else if (meas.angVelValid()) {
            applyAngVelToState(sys, state, processModel, meas);
        } else {
            // unusually, the measurement is totally invalid. Just normalize and
            // go on.
            state.postCorrect();
        }
    }
} // namespace uvbi
} // namespace videotracker
