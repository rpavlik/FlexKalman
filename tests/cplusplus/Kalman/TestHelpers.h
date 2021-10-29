/** @file
    @brief Header

    @date 2015-2019

    @author
    Ryan Pavlik
    <ryan.pavlik@collabora.com>

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
//
// SPDX-License-Identifer: Apache-2.0
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

// DO NOT include headers here! This is shared between the "combined" tests and
// the split-header tests...

namespace Eigen {
inline void outputQuat(std::ostream &os, Quaterniond const &q) {
    os.precision(8);
    os << "[" << q.w() << " (" << q.vec().transpose() << ")]";
}
inline std::ostream &operator<<(std::ostream &os, Quaterniond const &q) {
    outputQuat(os, q);
    return os;
}
} // namespace Eigen
