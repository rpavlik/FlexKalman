/** @file
    @brief Header

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
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

// Internal Includes
#include "CSV.h"
// #include "MathTypesC.h"
// #include "TimeValue.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace videotracker {
namespace util {
    struct DefaultGroupTag;
    /// A class that exists to make it easier to output composite data
    /// structures to CSV. You can overload `void
    /// operator<<(CellGroupProxy<RowProxyType, GroupTag> &group,
    /// YourThing const& d)`,
    /// doing `group << cell("Item", d.item)`, etc. then invoke it just by
    /// passing an object of that type to cellGroup(), which takes an optional
    /// first parameter, a prefix to appead to all the headings.
    template <typename RowProxyType, typename GroupTag = DefaultGroupTag>
    class CellGroupProxy {
      public:
        CellGroupProxy(RowProxyType &row, const char *groupPrefix)
            : row_(row), prefix_(groupPrefix){};
        CellGroupProxy(RowProxyType &row, std::string const &groupPrefix)
            : row_(row), prefix_(groupPrefix){};

        RowProxyType getRow() { return std::forward<RowProxyType>(row_); }

        std::string const &getPrefix() const { return prefix_; }

      private:
        RowProxyType &row_;
        const std::string prefix_;
    };

    /// Handles user results from overloading
    /// `operator<<(CellGroupProxy<RowProxyType, GroupTag> &group, TheirThing)`
    /// and
    /// sending in `cell` calls within it.
    template <typename RowProxyType, typename GroupTag, typename... CellArgs>
    inline CellGroupProxy<RowProxyType, GroupTag> &
    operator<<(CellGroupProxy<RowProxyType, GroupTag> &group,
               detail::Cell<CellArgs...> &&currentCell) {
        // cell.addHeaderPrefix(group.getPrefix());
        group.getRow().add(cell(group.getPrefix() + currentCell.getHeader(),
                                currentCell.getData()));
        return group;
    }

    namespace detail {
        template <typename PrefixType> struct PrefixTypeToArgType {
            using type = PrefixType;
        };
        template <> struct PrefixTypeToArgType<std::string> {
            using type = std::string const &;
        };
        /// Temporary object created by the cellGroup call, captures the string
        /// (literal) prefix, the data reference/type, and the group tag.
        template <typename T, typename PrefixType = const char *,
                  typename Tag = DefaultGroupTag>
        class CellGroup {
          public:
            using prefix_type = PrefixType;
            using prefix_arg_type =
                typename PrefixTypeToArgType<PrefixType>::type;
            CellGroup(prefix_arg_type prefix, T const &data)
                : prefix_(prefix), data_(data) {}

            prefix_arg_type getPrefix() const { return prefix_; }
            T const &getData() const { return data_; }

          private:
            PrefixType prefix_;
            T const &data_;
        };
#if 0
        template <typename T, typename Tag>
        class CellGroup<T, std::string, Tag> {
          public:
            using prefix_type = std::string;
            CellGroup(prefix_type const &prefix, T const &data)
                : prefix_(prefix), data_(data) {}

            prefix_type const &getPrefix() const { return prefix_; }
            T const &getData() const { return data_; }

          private:
            prefix_type prefix_;
            T const &data_;
        };
#endif

        /// Wrapper for RowProxy lvalue-reference and CellGroup:
        /// creates the specially-typed CellGroupProxy object that will activate
        /// the user's operator<< and ours in turn, and takes care of getting
        /// the return value correct for chained calls.
        template <typename Derived, typename T, typename PrefixType,
                  typename Tag>
        inline CSVRowProxy<Derived> &
        operator<<(CSVRowProxy<Derived> &row,
                   detail::CellGroup<T, PrefixType, Tag> const &group) {
            using RowProxyType = CSVRowProxy<Derived> &;
            auto proxy =
                CellGroupProxy<RowProxyType, Tag>(row, group.getPrefix());
            proxy << group.getData();
            return row;
        }

        /// Wrapper for RowProxy rvalue-reference and CellGroup.
        /// creates the specially-typed CellGroupProxy object that will activate
        /// the user's operator<< and ours in turn, and takes care of getting
        /// the return value correct for chained calls.
        template <typename Derived, typename T, typename PrefixType,
                  typename Tag>
        inline CSVRowProxy<Derived> &&
        operator<<(CSVRowProxy<Derived> &&row,
                   detail::CellGroup<T, PrefixType, Tag> const &group) {
            using RowProxyType = CSVRowProxy<Derived> &&;
            auto proxy =
                CellGroupProxy<RowProxyType, Tag>{row, group.getPrefix()};
            proxy << group.getData();
            // Must move this because of return type
            return std::move(row);
        }

    } // namespace detail

    /// Helper function to create a cell group with a group header prefix.
    template <typename T>
    inline detail::CellGroup<T> cellGroup(const char *groupHeader,
                                          T const &data) {
        return detail::CellGroup<T>{groupHeader, data};
    }

    /// Helper function to create a cell group with a group header prefix.
    template <typename T>
    inline detail::CellGroup<T, std::string>
    cellGroup(std::string const &groupHeader, T const &data) {
        return detail::CellGroup<T, std::string>{groupHeader, data};
    }
    /// Helper function to create a cell group with a group header prefix and
    /// explicit (non-default) tag.
    template <typename Tag, typename T>
    inline detail::CellGroup<T, const char *, Tag>
    cellGroup(const char *groupHeader, T const &data,
              Tag * /*dummy*/ = static_cast<Tag *>(nullptr)) {
        return detail::CellGroup<T, const char *, Tag>{groupHeader, data};
    }

    /// Helper function to create a cell group with a group header prefix and
    /// explicit (non-default) tag.
    template <typename Tag, typename T>
    inline detail::CellGroup<T, std::string, Tag>
    cellGroup(std::string const &groupHeader, T const &data,
              Tag * /*dummy*/ = static_cast<Tag *>(nullptr)) {
        return detail::CellGroup<T, std::string, Tag>{groupHeader, data};
    }
    /// Helper function to create a cell group with no group header prefix -
    /// warning: column headers must be unique!
    template <typename T> inline detail::CellGroup<T> cellGroup(T const &data) {
        return detail::CellGroup<T>{"", data};
    }

    /// Helper function to create a cell group with no group header prefix and
    /// explicit (non-default) tag - warning: column headers must be unique!
    template <typename Tag, typename T>
    inline detail::CellGroup<T, const char *, Tag>
    cellGroup(T const &data, Tag * /*dummy*/ = static_cast<Tag *>(nullptr)) {
        return detail::CellGroup<T, const char *, Tag>{"", data};
    }
#if 0
    template <typename T>
    inline void operator<<(CellGroupProxy<T, DefaultGroupTag> &group,
                           TimeValue const &tv) {
        group << cell("seconds", tv.seconds)
              << cell("microseconds", tv.microseconds);
    }
    struct AbbreviatedTimeMemberFieldsTag;
    template <typename T>
    inline void
    operator<<(CellGroupProxy<T, AbbreviatedTimeMemberFieldsTag> &group,
               TimeValue const &tv) {
        group << cell("sec", tv.seconds) << cell("usec", tv.microseconds);
    }

    struct DecimalTimeFieldTag;
    template <typename T>
    inline void operator<<(CellGroupProxy<T, DecimalTimeFieldTag> &group,
                           TimeValue const &tv) {

        group << cell("seconds", time::toDecimalString(tv));
    }

    template <typename T>
    inline void operator<<(CellGroupProxy<T, DefaultGroupTag> &group,
                           UVBI_Vec3 const &v) {
        group << cell("x", v.data[0]) << cell("y", v.data[1])
              << cell("z", v.data[2]);
    }

    template <typename T>
    inline void operator<<(CellGroupProxy<T, DefaultGroupTag> &group,
                           UVBI_Quaternion const &q) {
        group << cell("qw", uvbiQuatGetW(&q)) << cell("qx", uvbiQuatGetX(&q))
              << cell("qy", uvbiQuatGetY(&q)) << cell("qz", uvbiQuatGetZ(&q));
    }
#endif
#ifdef EIGEN_WORLD_VERSION
    template <typename T, typename Scalar>
    inline void operator<<(CellGroupProxy<T, DefaultGroupTag> &group,
                           Eigen::Matrix<Scalar, 2, 1> const &v) {
        group << cell("x", v.x()) << cell("y", v.y());
    }

    template <typename T>
    inline void operator<<(CellGroupProxy<T, DefaultGroupTag> &group,
                           Eigen::Vector3d const &v) {
        group << cell("x", v.x()) << cell("y", v.y()) << cell("z", v.z());
    }

    template <typename T>
    inline void operator<<(CellGroupProxy<T, DefaultGroupTag> &group,
                           Eigen::Quaterniond const &q) {
        group << cell("qw", q.w()) << cell("qx", q.x()) << cell("qy", q.y())
              << cell("qz", q.z());
    }
#endif

} // namespace util
} // namespace videotracker
