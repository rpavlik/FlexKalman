/** @file
    @brief Header for assert macros.

    Include guards intentionally omitted, to allow re-inclusion with different
   options.

    Assertions can either do nothing, call an assert handler on failure that
   prints details to stderr, or call your compiler system's assert.

    - Define `VIDEOTRACKER_DISABLE_ASSERTS` before including this file to
   forcibly disable all asserts.
    - By default, debug builds will use the standard assert method, and release
   builds will do nothing.
    - To unconditionally (debug and release) enable the custom assert handler,
   define `VIDEOTRACKER_ENABLE_ASSERT_HANDLER`-
   note that this source is not present...
    - To enable the custom assert handler for debug builds only (leaving asserts
   as no-ops in release builds), define
   `VIDEOTRACKER_ENABLE_ASSERT_DEBUG_HANDLER` - note that this source is not
   present...


    @date 2015

    @author
    Ryan Pavlik (incorporating some code modified from Boost)
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
//
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// Based on the VRPN vrpn_Assert.h file,
// itself including code adapted from the following Boost Software License v1.0
// sources:
//  - <boost/current_function.hpp>
//  - <boost/assert.hpp>

// Undefine macro for safe multiple inclusion
#undef VIDEOTRACKER_CURRENT_FUNCTION

// ---------------------------------------------------------- //
// Begin code adapted from <boost/current_function.hpp>
// at revision 5d353ad2b of the boost.assert repository
// https://github.com/boostorg/assert/blob/5d353ad2b92208c6ca300f4b47fdf04c87a8a593/include/boost/current_function.hpp
//
// Original notice follows:
//
//  Copyright (c) 2002 Peter Dimov and Multi Media Ltd.
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  http://www.boost.org/libs/assert/current_function.html
//
#if defined(__GNUC__) || (defined(__MWERKS__) && (__MWERKS__ >= 0x3000)) ||    \
    (defined(__ICC) && (__ICC >= 600)) || defined(__ghs__)

#define VIDEOTRACKER_CURRENT_FUNCTION __PRETTY_FUNCTION__

#elif defined(__DMC__) && (__DMC__ >= 0x810)

#define VIDEOTRACKER_CURRENT_FUNCTION __PRETTY_FUNCTION__

#elif defined(__FUNCSIG__)

#define VIDEOTRACKER_CURRENT_FUNCTION __FUNCSIG__

#elif (defined(__INTEL_COMPILER) && (__INTEL_COMPILER >= 600)) ||              \
    (defined(__IBMCPP__) && (__IBMCPP__ >= 500))

#define VIDEOTRACKER_CURRENT_FUNCTION __FUNCTION__

#elif defined(__BORLANDC__) && (__BORLANDC__ >= 0x550)

#define VIDEOTRACKER_CURRENT_FUNCTION __FUNC__

#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901)

#define VIDEOTRACKER_CURRENT_FUNCTION __func__

#elif defined(__cplusplus) && (__cplusplus >= 201103)

#define VIDEOTRACKER_CURRENT_FUNCTION __func__

#else

#define VIDEOTRACKER_CURRENT_FUNCTION "(unknown)"

#endif

// End code adapted from <boost/current_function.hpp>
// ---------------------------------------------------------- //

// ---------------------------------------------------------- //
// Begin code adapted from <boost/assert.hpp>
// at revision 5d353ad2b of the boost.assert repository
// https://github.com/boostorg/assert/blob/5d353ad2b92208c6ca300f4b47fdf04c87a8a593/include/boost/assert.hpp
//
// Original notice follows:
//
//  Copyright (c) 2001, 2002 Peter Dimov and Multi Media Ltd.
//  Copyright (c) 2007, 2014 Peter Dimov
//  Copyright (c) Beman Dawes 2011
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  Note: There are no include guards. This is intentional.
//
//  See http://www.boost.org/libs/assert/assert.html for documentation.
//

//
// VIDEOTRACKER_ASSERT, VIDEOTRACKER_ASSERT_MSG
//

#undef VIDEOTRACKER_ASSERT
#undef VIDEOTRACKER_ASSERT_MSG

#if defined(VIDEOTRACKER_DISABLE_ASSERTS) ||                                   \
    (defined(VIDEOTRACKER_ENABLE_ASSERT_DEBUG_HANDLER) && defined(NDEBUG))

#define VIDEOTRACKER_ASSERT(expr) ((void)0)
#define VIDEOTRACKER_ASSERT_MSG(expr, msg) ((void)0)

#elif defined(VIDEOTRACKER_ENABLE_ASSERT_HANDLER) ||                           \
    (defined(VIDEOTRACKER_ENABLE_ASSERT_DEBUG_HANDLER) && !defined(NDEBUG))

/// @todo implementation of UVBI_LIKELY
#ifndef UVBI_LIKELY
#define UVBI_LIKELY(X) (X)
#endif

namespace videotracker {
UVBI_API void assertion_failed(char const *expr, char const *function,
                               char const *file, long line);
UVBI_API void assertion_failed_msg(char const *expr, char const *msg,
                                   char const *function, char const *file,
                                   long line);
} // namespace videotracker

#define VIDEOTRACKER_ASSERT(expr)                                              \
    (UVBI_LIKELY(!!(expr))                                                     \
         ? ((void)0)                                                           \
         : ::vrpn::assertion_failed(#expr, VIDEOTRACKER_CURRENT_FUNCTION,      \
                                    __FILE__, __LINE__))
#define VIDEOTRACKER_ASSERT_MSG(expr, msg)                                     \
    (UVBI_LIKELY(!!(expr))                                                     \
         ? ((void)0)                                                           \
         : ::vrpn::assertion_failed_msg(                                       \
               #expr, msg, VIDEOTRACKER_CURRENT_FUNCTION, __FILE__, __LINE__))

#else

#include <assert.h> // .h to support old libraries w/o <cassert> - effect is the same

#define VIDEOTRACKER_ASSERT(expr) assert(expr)
#define VIDEOTRACKER_ASSERT_MSG(expr, msg) assert((expr) && (msg))

#endif

//
// VIDEOTRACKER_VERIFY, VIDEOTRACKER_VERIFY_MSG
//

#undef VIDEOTRACKER_VERIFY
#undef VIDEOTRACKER_VERIFY_MSG

#if defined(VIDEOTRACKER_DISABLE_ASSERTS) ||                                   \
    (!defined(VIDEOTRACKER_ENABLE_ASSERT_HANDLER) && defined(NDEBUG))

#define VIDEOTRACKER_VERIFY(expr) ((void)(expr))
#define VIDEOTRACKER_VERIFY_MSG(expr, msg) ((void)(expr))

#else

#define VIDEOTRACKER_VERIFY(expr) VIDEOTRACKER_ASSERT(expr)
#define VIDEOTRACKER_VERIFY_MSG(expr, msg) VIDEOTRACKER_ASSERT_MSG(expr, msg)

#endif

// End code adapted from <boost/assert.hpp>
// --

// ---------
// Documentation
/** @def VIDEOTRACKER_CURRENT_FUNCTION
    @brief Expands to the special preprocessor macro providing a useful
    description of the current function, where available.
*/
/** @def VIDEOTRACKER_ASSERT(expr)
    @brief Asserts the truth of @p expr according to the configuration of
    UVBI_Assert.h at the time of inclusion. If not asserting, does not evaluate
    expression.
*/
/** @def VIDEOTRACKER_ASSERT_MSG(expr, msg)
    @brief Like VIDEOTRACKER_ASSERT(expr) but allows specification of a message
   to be included in the case of a failed assertion.
*/
/** @def VIDEOTRACKER_VERIFY(expr)
    @brief Typically forwards to VIDEOTRACKER_ASSERT, but in cases where
   VIDEOTRACKER_ASSERT would expand to nothing (not evaluating the expression),
   VIDEOTRACKER_VERIFY evaluates the expression but discards the result.
*/
/** @def VIDEOTRACKER_VERIFY_MSG(expr, msg)
    @brief Like VIDEOTRACKER_VERIFY(expr) but allows specification of a message
   to be included in the case of a failed assertion.
*/
