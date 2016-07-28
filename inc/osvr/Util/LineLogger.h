/** @file
    @brief Header

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com>

*/

// Copyright 2016 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// 	http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDED_LineLogger_h_GUID_743865A8_E989_4A87_82D9_9BBF3E4C199D
#define INCLUDED_LineLogger_h_GUID_743865A8_E989_4A87_82D9_9BBF3E4C199D

// Internal Includes
#include <osvr/Util/Export.h>
#include <osvr/Util/LogLevel.h>

// Library/third-party includes
// - none

// Standard includes
#include <memory>  // for std::unique_ptr
#include <sstream> // for std::ostringstream
#include <string>  // for std::string
#include <utility> // for std::forward

// Forward declaration
namespace spdlog {
namespace details {
    struct log_msg;
    class line_logger;
} // namespace details
} // namespace spdlog

namespace osvr {
namespace util {
    namespace log {

        // Forward declaration
        class Logger;

        namespace detail {

            /**
             * @brief A wrapper class for spdlog::details::line_logger.
             */
            class LineLogger {
              public:
                OSVR_UTIL_EXPORT
                LineLogger(spdlog::details::line_logger &&line_logger);

                /**
                 * @brief Move-only.
                 */
                //@{
                OSVR_UTIL_EXPORT LineLogger(const LineLogger &other) = delete;
                OSVR_UTIL_EXPORT LineLogger &
                operator=(const LineLogger &) = delete;
                OSVR_UTIL_EXPORT LineLogger &operator=(LineLogger &&) = delete;
                OSVR_UTIL_EXPORT LineLogger(LineLogger &&other);
                //@}

                /**
                 * @brief Log the message using the callback logger.
                 */
                OSVR_UTIL_EXPORT ~LineLogger();

                /**
                 * @brief Support for format string with variadic args.
                 */
                OSVR_UTIL_EXPORT void write(const char *what);

                template <typename... Args>
                OSVR_UTIL_EXPORT void write(const char *fmt, Args &&... args);

/**
 * @brief Support for operator<<
 *
 * \name Stream operators
 */
//@{
#if 0
                OSVR_UTIL_EXPORT LineLogger &operator<<(const char *what);
                OSVR_UTIL_EXPORT LineLogger &
                operator<<(const std::string &what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(int what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(unsigned int what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(long what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(unsigned long what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(long long what);
                OSVR_UTIL_EXPORT LineLogger &
                operator<<(unsigned long long what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(double what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(long double what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(float what);
                OSVR_UTIL_EXPORT LineLogger &operator<<(char what);

                template <typename T> LineLogger &operator<<(T &&what) {
                    std::ostringstream ss;
                    ss.operator<<(std::forward<T>(what));
                    this->operator<<(ss.str());
                    return *this;
                }
#endif
                //@}

                /// Same as the operator<< with equivalent params, but doesn't
                /// participate in that overload resolution nor will it be
                /// captured by the perfect forwarder.
                OSVR_UTIL_EXPORT LineLogger &append(const std::string &what);

                class StreamProxy {
                  public:
                    StreamProxy(LineLogger &lineLogger)
                        : lineLogger_(lineLogger), os_(new std::ostringstream) {
                    }

                    /// destructor appends the finished stringstream at the end
                    /// of the expression.
                    ~StreamProxy() {
                        if (active_ && os_) {
                            lineLogger_.append(os_->str());
                        }
                    }

                    /// move construction
                    StreamProxy(StreamProxy &&other)
                        : lineLogger_(other.lineLogger_),
                          os_(std::move(other.os_)), active_(other.active_) {
                        other.active_ = false;
                    }

                    StreamProxy(StreamProxy const &) = delete;
                    StreamProxy &operator=(StreamProxy const &) = delete;

                    operator std::ostream &() { return (*os_); }

                    template <typename T> std::ostream &operator<<(T &&what) {
                        (*os_) << std::forward<T>(what);
                        return (*os_);
                    }

                  private:
                    LineLogger &lineLogger_;
                    std::unique_ptr<std::ostringstream> os_;
                    bool active_ = true;
                };

                template <typename T> StreamProxy operator<<(T &&what) {
                    StreamProxy proxy(*this);
                    proxy << std::forward<T>(what);
                    return proxy;
                }

                OSVR_UTIL_EXPORT void disable();
                OSVR_UTIL_EXPORT bool is_enabled() const;

              private:
                std::unique_ptr<spdlog::details::line_logger> lineLogger_;
            };

        } // namespace detail

    } // namespace log

} // namespace util
} // namespace osvr

#endif // INCLUDED_LineLogger_h_GUID_743865A8_E989_4A87_82D9_9BBF3E4C199D