/** @file
    @brief Header for assert macros.

    Include guards intentionally omitted, to allow re-inclusion with different
   options.

    Assertions can either do nothing, call an assert handler on failure that
   prints details to stderr, or call your compiler system's assert.

    - Define `UVBI_DISABLE_ASSERTS` before including this file to forcibly
   disable all asserts.
    - By default, debug builds will use the standard assert method, and release
   builds will do nothing.
    - To unconditionally (debug and release) enable the custom assert handler,
   define `UVBI_ENABLE_ASSERT_HANDLER`
    - To enable the custom assert handler for debug builds only (leaving asserts
   as no-ops in release builds), define `UVBI_ENABLE_ASSERT_DEBUG_HANDLER` -
   note that this source is not present...


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
#undef UVBI_CURRENT_FUNCTION

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

#define UVBI_CURRENT_FUNCTION __PRETTY_FUNCTION__

#elif defined(__DMC__) && (__DMC__ >= 0x810)

#define UVBI_CURRENT_FUNCTION __PRETTY_FUNCTION__

#elif defined(__FUNCSIG__)

#define UVBI_CURRENT_FUNCTION __FUNCSIG__

#elif (defined(__INTEL_COMPILER) && (__INTEL_COMPILER >= 600)) ||              \
    (defined(__IBMCPP__) && (__IBMCPP__ >= 500))

#define UVBI_CURRENT_FUNCTION __FUNCTION__

#elif defined(__BORLANDC__) && (__BORLANDC__ >= 0x550)

#define UVBI_CURRENT_FUNCTION __FUNC__

#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901)

#define UVBI_CURRENT_FUNCTION __func__

#elif defined(__cplusplus) && (__cplusplus >= 201103)

#define UVBI_CURRENT_FUNCTION __func__

#else

#define UVBI_CURRENT_FUNCTION "(unknown)"

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
// UVBI_ASSERT, UVBI_ASSERT_MSG
//

#undef UVBI_ASSERT
#undef UVBI_ASSERT_MSG

#if defined(UVBI_DISABLE_ASSERTS) ||                                           \
    (defined(UVBI_ENABLE_ASSERT_DEBUG_HANDLER) && defined(NDEBUG))

#define UVBI_ASSERT(expr) ((void)0)
#define UVBI_ASSERT_MSG(expr, msg) ((void)0)

#elif defined(UVBI_ENABLE_ASSERT_HANDLER) ||                                   \
    (defined(UVBI_ENABLE_ASSERT_DEBUG_HANDLER) && !defined(NDEBUG))

/// @todo implementation of UVBI_LIKELY
#ifndef UVBI_LIKELY
#define UVBI_LIKELY(X) (X)
#endif

namespace uvbi {
UVBI_API void assertion_failed(char const *expr, char const *function,
                               char const *file, long line);
UVBI_API void assertion_failed_msg(char const *expr, char const *msg,
                                   char const *function, char const *file,
                                   long line);
} // namespace uvbi

#define UVBI_ASSERT(expr)                                                      \
    (UVBI_LIKELY(!!(expr))                                                     \
         ? ((void)0)                                                           \
         : ::vrpn::assertion_failed(#expr, UVBI_CURRENT_FUNCTION, __FILE__,    \
                                    __LINE__))
#define UVBI_ASSERT_MSG(expr, msg)                                             \
    (UVBI_LIKELY(!!(expr))                                                     \
         ? ((void)0)                                                           \
         : ::vrpn::assertion_failed_msg(#expr, msg, UVBI_CURRENT_FUNCTION,     \
                                        __FILE__, __LINE__))

#else

#include <assert.h> // .h to support old libraries w/o <cassert> - effect is the same

#define UVBI_ASSERT(expr) assert(expr)
#define UVBI_ASSERT_MSG(expr, msg) assert((expr) && (msg))

#endif

//
// UVBI_VERIFY, UVBI_VERIFY_MSG
//

#undef UVBI_VERIFY
#undef UVBI_VERIFY_MSG

#if defined(UVBI_DISABLE_ASSERTS) ||                                           \
    (!defined(UVBI_ENABLE_ASSERT_HANDLER) && defined(NDEBUG))

#define UVBI_VERIFY(expr) ((void)(expr))
#define UVBI_VERIFY_MSG(expr, msg) ((void)(expr))

#else

#define UVBI_VERIFY(expr) UVBI_ASSERT(expr)
#define UVBI_VERIFY_MSG(expr, msg) UVBI_ASSERT_MSG(expr, msg)

#endif

// End code adapted from <boost/assert.hpp>
// --

// ---------
// Documentation
/** @def UVBI_CURRENT_FUNCTION
    @brief Expands to the special preprocessor macro providing a useful
    description of the current function, where available.
*/
/** @def UVBI_ASSERT(expr)
    @brief Asserts the truth of @p expr according to the configuration of
    UVBI_Assert.h at the time of inclusion. If not asserting, does not evaluate
    expression.
*/
/** @def UVBI_ASSERT_MSG(expr, msg)
    @brief Like UVBI_ASSERT(expr) but allows specification of a message to be
    included in the case of a failed assertion.
*/
/** @def UVBI_VERIFY(expr)
    @brief Typically forwards to UVBI_ASSERT, but in cases where UVBI_ASSERT
    would expand to nothing (not evaluating the expression), UVBI_VERIFY
    evaluates the expression but discards the result.
*/
/** @def UVBI_VERIFY_MSG(expr, msg)
    @brief Like UVBI_VERIFY(expr) but allows specification of a message to be
    included in the case of a failed assertion.
*/
