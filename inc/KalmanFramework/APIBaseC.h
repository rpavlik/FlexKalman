/** @file
    @brief Header providing basic C macros for defining API headers.

    Must be c-safe!

    @date 2014

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

/*
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
*/

#ifndef INCLUDED_APIBaseC_h_GUID_C5A2E769_2ADC_429E_D250_DF0883E6E5DB
#define INCLUDED_APIBaseC_h_GUID_C5A2E769_2ADC_429E_D250_DF0883E6E5DB

#ifdef __cplusplus
#define KALMANFRAMEWORK_C_ONLY(X)
#define KALMANFRAMEWORK_CPP_ONLY(X) X
#define KALMANFRAMEWORK_EXTERN_C_BEGIN extern "C" {
#define KALMANFRAMEWORK_EXTERN_C_END }
#define KALMANFRAMEWORK_INLINE inline
#else
#define KALMANFRAMEWORK_C_ONLY(X) X
#define KALMANFRAMEWORK_CPP_ONLY(X)
#define KALMANFRAMEWORK_EXTERN_C_BEGIN
#define KALMANFRAMEWORK_EXTERN_C_END
#ifdef _MSC_VER
#define KALMANFRAMEWORK_INLINE static __inline
#else
#define KALMANFRAMEWORK_INLINE static inline
#endif
#endif

#endif
