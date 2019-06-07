/** @file
    @brief Implementation

    @date 2014

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2014 Sensics, Inc.
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

// Internal Includes
#include <KalmanFramework/TimeValueC.h>

// Library/third-party includes
// - none

// Standard includes
#include <ratio>

#if defined(KALMANFRAMEWORK_HAVE_STRUCT_TIMEVAL_IN_SYS_TIME_H)
#include <sys/time.h>
typedef time_t tv_seconds_type;
typedef suseconds_t tv_microseconds_type;
#elif defined(KALMANFRAMEWORK_HAVE_STRUCT_TIMEVAL_IN_WINSOCK2_H)
//#include <winsock2.h>
typedef long tv_seconds_type;
typedef long tv_microseconds_type;
#endif

#define OSVR_USEC_PER_SEC std::micro::den;

void osvrTimeValueNormalize(OSVR_TimeValue *tv) {
    if (!tv) {
        return;
    }
    const OSVR_TimeValue_Microseconds rem =
        tv->microseconds / OSVR_USEC_PER_SEC;
    tv->seconds += rem;
    tv->microseconds -= rem * OSVR_USEC_PER_SEC;
    /* By here, abs(microseconds) < OSVR_USEC_PER_SEC:
       now let's get signs the same. */
    if (tv->seconds > 0 && tv->microseconds < 0) {
        tv->seconds--;
        tv->microseconds += OSVR_USEC_PER_SEC;
    } else if (tv->seconds < 0 && tv->microseconds > 0) {
        tv->seconds++;
        tv->microseconds -= OSVR_USEC_PER_SEC;
    }
}

void osvrTimeValueSum(OSVR_TimeValue *tvA, const OSVR_TimeValue *tvB) {
    if (!tvA || !tvB) {
        return;
    }
    tvA->seconds += tvB->seconds;
    tvA->microseconds += tvB->microseconds;
    osvrTimeValueNormalize(tvA);
}

void osvrTimeValueDifference(OSVR_TimeValue *tvA, const OSVR_TimeValue *tvB) {
    if (!tvA || !tvB) {
        return;
    }
    tvA->seconds -= tvB->seconds;
    tvA->microseconds -= tvB->microseconds;
    osvrTimeValueNormalize(tvA);
}

template <typename T> inline int numcmp(T a, T b) {
    return (a == b) ? 0 : (a < b ? -1 : 1);
}

int osvrTimeValueCmp(const OSVR_TimeValue *tvA, const OSVR_TimeValue *tvB) {
    if (!tvA || !tvB) {
        return 0;
    }
    auto major = numcmp(tvA->seconds, tvB->seconds);
    return (major != 0) ? major : numcmp(tvA->microseconds, tvB->microseconds);
}

void osvrTimeValueGetNow(OSVR_TimeValue *dest) {
    timeval tv;
    gettimeofday(&tv, nullptr);
    osvrStructTimevalToTimeValue(dest, &tv);
}

void osvrTimeValueToStructTimeval(timeval *dest, const OSVR_TimeValue *src) {
    if (!dest || !src) {
        return;
    }
    dest->tv_sec = tv_seconds_type(src->seconds);
    dest->tv_usec = tv_microseconds_type(src->microseconds);
}

void osvrStructTimevalToTimeValue(OSVR_TimeValue *dest, const timeval *src) {
    if (!dest || !src) {
        return;
    }
    dest->seconds = src->tv_sec;
    dest->microseconds = src->tv_usec;
    osvrTimeValueNormalize(dest);
}
