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

#include <iostream>
#include <sstream>
#include <string>

#include "csv.hpp"

enum class EntryType { Orientation, Position, PositionLeverArm };

struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};
inline Eigen::Quaterniond parseQuat(csv::CSVRow &row) {
    return Eigen::Quaterniond(row["w"].get<double>(), row["x"].get<double>(),
                              row["y"].get<double>(), row["z"].get<double>());
}

inline Eigen::Vector3d parseVec3(csv::CSVRow &row) {
    return Eigen::Vector3d(row["x"].get<double>(), row["y"].get<double>(),
                           row["z"].get<double>());
}
inline Eigen::Vector3d parseOffsetVec3(csv::CSVRow &row) {
    return Eigen::Vector3d(row["off_x"].get<double>(),
                           row["off_y"].get<double>(),
                           row["off_z"].get<double>());
}

inline Eigen::Quaterniond parseRotVec(csv::CSVRow &row) {
    Eigen::Vector3d rotVec = parseVec3(row);

    return Eigen::Quaterniond(
        Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()));
}

inline Pose parseRealPose(csv::CSVRow &row) {
    return {Eigen::Vector3d(row["real_x"].get<double>(),
                            row["real_y"].get<double>(),
                            row["real_z"].get<double>()),
            Eigen::Quaterniond(
                row["real_qw"].get<double>(), row["real_qx"].get<double>(),
                row["real_qy"].get<double>(), row["real_qz"].get<double>())};
}

inline bool parseEntry(csv::CSVRow &row, EntryType &type, double &dt,
                       Eigen::Quaterniond &orientation,
                       Eigen::Vector3d &position, Eigen::Vector3d &offset) {
    auto typeName = row["type"].get<csv::string_view>();
    dt = row["dt"].get<double>();
    if (typeName == "quat") {
        orientation = parseQuat(row);
        type = EntryType::Orientation;
        return true;
    }
    if (typeName == "rotvec") {
        orientation = parseRotVec(row);
        type = EntryType::Orientation;
        return true;
    }

    if (typeName == "pos") {
        position = parseVec3(row);
        type = EntryType::Position;
        return true;
    }
    if (typeName == "posLever") {
        position = parseVec3(row);
        offset = parseOffsetVec3(row);
        type = EntryType::PositionLeverArm;
        return true;
    }
    throw std::runtime_error("Unrecognized type " + std::string(typeName));
}
