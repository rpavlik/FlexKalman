/** @file
    @brief Header defining a dependency-free, cross-platform substitute for
   struct timeval

    Must be c-safe!

    @date 2014

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

/*
// Copyright 2014 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once

/* Internal Includes */
#include "APIBaseC.h"
// #include <FlexKalman/Export.h>

/* Library/third-party includes */
/* none */

/* Standard includes */
#include <stdint.h>

#if defined(_WIN32) && !defined(__MINGW32__)
#define KALMANFRAMEWORK_HAVE_STRUCT_TIMEVAL_IN_WINSOCK2_H
#include <Winsock2.h>
#else
#define KALMANFRAMEWORK_HAVE_STRUCT_TIMEVAL_IN_SYS_TIME_H
#include <sys/time.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup UtilTime Timestamp interaction
    @ingroup Util

    This provides a level of interoperability with struct timeval on systems
    with that facility. It provides a neutral representation with sufficiently
    large types.

    For C++ code, use of std::chrono or boost::chrono instead is recommended.

    Note that these time values may not necessarily correlate between processes
   so should not be used to estimate or measure latency, etc.

    @{
*/

/** @brief The signed integer type storing the seconds in a struct
    UVBI_TimeValue */
typedef int64_t UVBI_TimeValue_Seconds;
/** @brief The signed integer type storing the microseconds in a struct
    UVBI_TimeValue */
typedef int32_t UVBI_TimeValue_Microseconds;

/** @brief Standardized, portable parallel to struct timeval for representing
   both absolute times and time intervals.

   Where interpreted as an absolute time, its meaning is to be considered the
   same as that of the POSIX struct timeval:
   time since 00:00 Coordinated Universal Time (UTC), January 1, 1970.

   For best results, please keep normalized. Output of all functions here
   is normalized.
   */
typedef struct UVBI_TimeValue {
    /** @brief Seconds portion of the time value. */
    UVBI_TimeValue_Seconds seconds;
    /** @brief Microseconds portion of the time value. */
    UVBI_TimeValue_Microseconds microseconds;
} UVBI_TimeValue;

/** @brief Gets the current time in the TimeValue. Parallel to gettimeofday. */
void uvbiTimeValueGetNow(UVBI_TimeValue *dest);

struct timeval; /* forward declaration */

/** @brief Converts from a TimeValue struct to your system's struct timeval.

    @param dest Pointer to an empty struct timeval for your platform.
    @param src A pointer to an UVBI_TimeValue you'd like to convert from.

    If either parameter is NULL, the function will return without doing
   anything.
*/
void uvbiTimeValueToStructTimeval(struct timeval *dest,
                                  const UVBI_TimeValue *src);

/** @brief Converts from a TimeValue struct to your system's struct timeval.
    @param dest An UVBI_TimeValue destination pointer.
    @param src Pointer to a struct timeval you'd like to convert from.

    The result is normalized.

    If either parameter is NULL, the function will return without doing
   anything.
*/
void osvrStructTimevalToTimeValue(UVBI_TimeValue *dest,
                                  const struct timeval *src);

/** @brief "Normalizes" a time value so that the absolute number of microseconds
    is less than 1,000,000, and that the sign of both components is the same.

    @param tv Address of a struct TimeValue to normalize in place.

    If the given pointer is NULL, this function returns without doing anything.
*/
void uvbiTimeValueNormalize(UVBI_TimeValue *tv);

/** @brief Sums two time values, replacing the first with the result.

    @param tvA Destination and first source.
    @param tvB second source

    If a given pointer is NULL, this function returns without doing anything.

    Both parameters are expected to be in normalized form.
*/
void uvbiTimeValueSum(UVBI_TimeValue *tvA, const UVBI_TimeValue *tvB);

/** @brief Computes the difference between two time values, replacing the first
    with the result.

    Effectively, `*tvA = *tvA - *tvB`

    @param tvA Destination and first source.
    @param tvB second source

    If a given pointer is NULL, this function returns without doing anything.

    Both parameters are expected to be in normalized form.
*/
void uvbiTimeValueDifference(UVBI_TimeValue *tvA, const UVBI_TimeValue *tvB);

/** @brief  Compares two time values (assumed to be normalized), returning
    the same values as strcmp

    @return <0 if A is earlier than B, 0 if they are the same, and >0 if A
    is later than B.
*/
int uvbiTimeValueCmp(const UVBI_TimeValue *tvA, const UVBI_TimeValue *tvB);

#ifdef __cplusplus
} // extern "C"
#endif

/** @brief Compute the difference between the two time values, returning the
    duration as a double-precision floating-point number of seconds.

    Effectively, `ret = *tvA - *tvB`

    @param tvA first source.
    @param tvB second source
    @return Duration of timespan in seconds (floating-point)
*/
UVBI_INLINE double uvbiTimeValueDurationSeconds(const UVBI_TimeValue *tvA,
                                                const UVBI_TimeValue *tvB) {
    UVBI_TimeValue A = *tvA;
    uvbiTimeValueDifference(&A, tvB);
    double dt = A.seconds + A.microseconds / 1000000.0;
    return dt;
}

/** @brief True if A is later than B */
UVBI_INLINE bool uvbiTimeValueGreater(const UVBI_TimeValue *tvA,
                                      const UVBI_TimeValue *tvB) {
    if (!tvA || !tvB) {
        return false;
    }
    return ((tvA->seconds > tvB->seconds) ||
            ((tvA->seconds == tvB->seconds) &&
             (tvA->microseconds > tvB->microseconds)));
}

#ifdef __cplusplus

#include <cassert>
#include <cmath>

/// Returns true if the time value is normalized. Typically used in assertions.
inline bool uvbiTimeValueIsNormalized(const UVBI_TimeValue &tv) {
#ifdef _LIBCPP_VERSION
    // libcxx has only floating-point abs and has several, so it makes this
    // ambiguous otherwise
    return std::abs(double(tv.microseconds)) < 1000000 &&
#else
    return (std::abs(tv.microseconds) < 1000000) &&
#endif
           (
               // zeros can be paired with a positive or negative
               (tv.seconds == 0) || (tv.microseconds == 0) ||
               // if both non-zero, then both must have same sign.
               ((tv.seconds > 0) == (tv.microseconds > 0)));
}

/// True if A is later than B
inline bool uvbiTimeValueGreater(const UVBI_TimeValue &tvA,
                                 const UVBI_TimeValue &tvB) {
    assert(uvbiTimeValueIsNormalized(tvA) &&
           "First timevalue argument to comparison was not normalized!");
    assert(uvbiTimeValueIsNormalized(tvB) &&
           "Second timevalue argument to comparison was not normalized!");
    return (tvA.seconds > tvB.seconds) ||
           (tvA.seconds == tvB.seconds && tvA.microseconds > tvB.microseconds);
}

/// Operator > overload for time values
inline bool operator>(const UVBI_TimeValue &tvA, const UVBI_TimeValue &tvB) {
    return uvbiTimeValueGreater(tvA, tvB);
}

/// Operator < overload for time values
inline bool operator<(const UVBI_TimeValue &tvA, const UVBI_TimeValue &tvB) {
    // Change the order of arguments before forwarding.
    return uvbiTimeValueGreater(tvB, tvA);
}

/// Operator == overload for time values
inline bool operator==(const UVBI_TimeValue &tvA, const UVBI_TimeValue &tvB) {
    assert(
        uvbiTimeValueIsNormalized(tvA) &&
        "First timevalue argument to equality comparison was not normalized!");
    assert(
        uvbiTimeValueIsNormalized(tvB) &&
        "Second timevalue argument to equality comparison was not normalized!");
    return (tvA.seconds == tvB.seconds) &&
           (tvA.microseconds == tvB.microseconds);
}
/// Operator == overload for time values
inline bool operator!=(const UVBI_TimeValue &tvA, const UVBI_TimeValue &tvB) {
    assert(uvbiTimeValueIsNormalized(tvA) && "First timevalue argument to "
                                             "inequality comparison was not "
                                             "normalized!");
    assert(uvbiTimeValueIsNormalized(tvB) && "Second timevalue argument to "
                                             "inequality comparison was not "
                                             "normalized!");
    return (tvA.seconds != tvB.seconds) ||
           (tvA.microseconds != tvB.microseconds);
}

#endif

/** @} */
