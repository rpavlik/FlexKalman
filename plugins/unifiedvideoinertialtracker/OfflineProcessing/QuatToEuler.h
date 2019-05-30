/** @file
    @brief Header

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

#ifndef INCLUDED_QuatToEuler_h_GUID_146FCB5A_7820_4B98_5BED_3223776E8CDF
#define INCLUDED_QuatToEuler_h_GUID_146FCB5A_7820_4B98_5BED_3223776E8CDF

// Internal Includes
#include "CSVCellGroup.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace osvr {
namespace util {
    struct QuatAsEulerTag;

    inline Eigen::Vector3d getEuler(Eigen::Quaterniond const &q) {
        Eigen::Vector3d ret;
        // This method seems terribly unreliable, but we're using it because we lack quatlib.
        return q.toRotationMatrix().eulerAngles(2, 0, 2);
    }

    template <typename T>
    inline void operator<<(CellGroupProxy<T, QuatAsEulerTag> &group,
                           Eigen::Vector3d const &euler) {
        group << cell("yaw", euler[0]) << cell("pitch", euler[1])
              << cell("roll", euler[2]);
    }
    template <typename T>
    inline void operator<<(CellGroupProxy<T, QuatAsEulerTag> &group,
                           Eigen::Quaterniond const &q) {
        group << getEuler(q);
    }
} // namespace util
} // namespace osvr

#endif // INCLUDED_QuatToEuler_h_GUID_146FCB5A_7820_4B98_5BED_3223776E8CDF
