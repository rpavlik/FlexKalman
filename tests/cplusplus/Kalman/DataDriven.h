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

#pragma once

#include "FlexKalman/FlexibleKalmanBase.h"

#include <memory>

class PoseFilterInterface {
  public:
    virtual ~PoseFilterInterface() = default;

    virtual void predict(double dt) = 0;
    virtual bool filterOrientation(Eigen::Quaterniond const &orientation,
                                   Eigen::Vector3d const &variance) = 0;
    virtual bool filterPosition(Eigen::Vector3d const &position,
                                Eigen::Vector3d const &variance) = 0;
    virtual bool filterPositionLeverArm(Eigen::Vector3d const &position,
                                        Eigen::Vector3d const &variance,
                                        Eigen::Vector3d const &offset) = 0;
    virtual void dumpState(std::ostream &os, bool showCovariance) const = 0;

    virtual Eigen::Vector3d getPosition() const = 0;

    virtual Eigen::Quaterniond getOrientation() const = 0;
    virtual Eigen::Isometry3d getPose() const = 0;

  protected:
    // must inherit
    PoseFilterInterface() = default;

  private:
};

using PoseFilterPtr = std::unique_ptr<PoseFilterInterface>;

PoseFilterPtr makeExpFilter();
PoseFilterPtr makeExternalQuatFilter();
