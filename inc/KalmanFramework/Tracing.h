/** @file
    @brief Header

    @date 2015

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
#include <KalmanFramework/Export.h>

// Library/third-party includes
// - none

// Standard includes
#include <cstdint>
#include <string>

namespace osvr {
namespace common {
    namespace tracing {
        typedef std::int64_t TraceBeginStamp;
#ifdef KALMANFRAMEWORK_COMMON_TRACING_ENABLED
        struct KALMANFRAMEWORK_EXPORT MainTracePolicy {
            static TraceBeginStamp begin(const char *text);
            static void end(const char *text, TraceBeginStamp stamp);
            static void mark(const char *text);
        };

        struct KALMANFRAMEWORK_EXPORT WorkerTracePolicy {
            static TraceBeginStamp begin(const char *text);
            static void end(const char *text, TraceBeginStamp stamp);
            static void mark(const char *text);
        };
        /// @brief Class template base for "region" tracing.
        template <typename TracePolicy> class TracingRegion {
          public:
            /// @brief Destructor, ending region
            ~TracingRegion() { TracePolicy::end(m_text, m_stamp); }

          protected:
            /// @brief Explicit constructor from string literal in subclass,
            /// starting region.
            explicit TracingRegion(const char text[]) : m_text(text) {
                m_stamp = TracePolicy::begin(m_text);
            }
            /// @brief noncopyable
            TracingRegion(TracingRegion const &) = delete;
            /// @brief nonassignable
            TracingRegion &operator=(TracingRegion const &) = delete;

          private:
            TraceBeginStamp m_stamp;
            const char *m_text;
        };
        template <typename Policy>
        inline void markConcatenation(const char *fixedString,
                                      std::string const &string) {
            Policy::mark((fixedString + string).c_str());
        }
#else  // KALMANFRAMEWORK_COMMON_TRACING_ENABLED ^^ // vv
       // !KALMANFRAMEWORK_COMMON_TRACING_ENABLED
        struct MainTracePolicy {
            static TraceBeginStamp begin(const char *) { return 0; }
            static void end(const char *, TraceBeginStamp) {}
            static void mark(const char *) {}
        };
        struct WorkerTracePolicy {
            static TraceBeginStamp begin(const char *) { return 0; }
            static void end(const char *, TraceBeginStamp) {}
            static void mark(const char *) {}
        };

        template <typename TracePolicy> class TracingRegion {
          protected:
            explicit TracingRegion(const char *) {}
        };
        inline TraceBeginStamp driverUpdateStart(std::string const &,
                                                 std::string const &) {
            return 0;
        }
        inline void driverUpdateEnd(TraceBeginStamp) {}
        template <typename Policy>
        inline void markConcatenation(const char *, std::string const &) {}
#endif // !KALMANFRAMEWORK_COMMON_TRACING_ENABLED

    } // namespace tracing
} // namespace common
} // namespace osvr
