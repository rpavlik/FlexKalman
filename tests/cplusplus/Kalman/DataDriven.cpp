/** @file
    @brief Header

    @date 2020

    @author
    Ryan Pavlik
    <ryan.pavlik@collabora.com>

*/

// Copyright 2020 Collabora, Ltd.
//
// SPDX-License-Identifier: BSL-1.0 OR Apache-2.0

#include "DataDriven.h"

#include "FlexKalman/AbsoluteOrientationMeasurement.h"
#include "FlexKalman/AbsolutePositionLeverArmMeasurement.h"
#include "FlexKalman/AbsolutePositionMeasurement.h"
#include "FlexKalman/FlexibleKalmanFilter.h"
#include "FlexKalman/FlexibleUnscentedCorrect.h"
#include "FlexKalman/PoseConstantVelocityGeneric.h"
#include "FlexKalman/PoseStateExponentialMap.h"

// using State = flexkalman::pose_exp_map::State;
// using ProcessModel = flexkalman::pose_exp_map::ConstantVelocityProcessModel;
using flexkalman::AbsoluteOrientationMeasurement;

namespace {

template <typename State,
          typename ProcessModel =
              flexkalman::PoseConstantVelocityGenericProcessModel<State>>
class PoseFilter : public PoseFilterInterface {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseFilter() = default;
    ~PoseFilter() override = default;

    void predict(double dt) override {
        flexkalman::predict(state, process, dt);
    }
    bool filterOrientation(Eigen::Quaterniond const &orientation,
                           Eigen::Vector3d const &variance) override {
        AbsoluteOrientationMeasurement meas{orientation, variance};
        return flexkalman::correctUnscented(state, meas);
    };
    bool filterPosition(Eigen::Vector3d const &position,
                        Eigen::Vector3d const &variance) override {
        flexkalman::AbsolutePositionMeasurement meas{position, variance};
        return flexkalman::correctUnscented(state, meas);
    }
    bool filterPositionLeverArm(Eigen::Vector3d const &position,
                                Eigen::Vector3d const &variance,
                                Eigen::Vector3d const &offset) override {
        flexkalman::AbsolutePositionLeverArmMeasurement meas{position, offset,
                                                             variance};
        return flexkalman::correctUnscented(state, meas);
    }
    void dumpState(std::ostream &os, bool showCovariance) const override {
        if (showCovariance) {
            os << state << "\n";
        } else {
            os << state.stateVector().transpose() << "\n";
        }
    }

    Eigen::Vector3d getPosition() const override { return state.position(); }

    Eigen::Quaterniond getOrientation() const override {
        return state.getQuaternion();
    }
    Eigen::Isometry3d getPose() const override { return state.getIsometry(); }

  private:
    State state;
    ProcessModel process;
};
} // namespace

PoseFilterPtr makeExpFilter() {
    return std::make_unique<PoseFilter<flexkalman::pose_exp_map::State/* , flexkalman::pose_exp_map::ConstantVelocityProcessModel */>>();
}
PoseFilterPtr makeExternalQuatFilter() {
    return std::
        make_unique<PoseFilter<flexkalman::pose_externalized_rotation::State /* , flexkalman::pose_externalized_rotation::ConstantVelocityProcessModel */>>();
}
