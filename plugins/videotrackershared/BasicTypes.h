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

#pragma once

// Internal Includes
// - none

// Library/third-party includes
#include <opencv2/core/core.hpp>

// Standard includes
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace osvr {
namespace vbtracker {

    typedef float Brightness;
    typedef std::deque<Brightness> BrightnessList;
    typedef std::pair<Brightness, Brightness> BrightnessMinMax;

    /// Pattern repeated almost twice
    typedef std::string LedPatternWrapped;

    typedef std::vector<cv::Point3f> Point3Vector;

    typedef std::vector<cv::Vec3d> Vec3Vector;

} // namespace vbtracker
} // namespace osvr
