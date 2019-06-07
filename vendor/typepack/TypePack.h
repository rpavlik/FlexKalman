/** @file
    @brief Generated single-file header containing all of a simple C++11
   template-parameter-pack-based metaprogramming library inspired by/based on
   Eric Niebler's `meta`


    NOTE: This is a generated single-file version of TypePack - do not edit
   directly! Instead, edit the individual source files and regenerate this with
   combine_headers.py.

    @date 2015-2019

    @author
    Ryan Pavlik
    <ryan.pavlik@collabora.com>

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>

    @author
    Eric Niebler
*/

// Copyright 2019 Collabora, Ltd.
// Copyright 2015-2017 Sensics, Inc.
//
// SPDX-License-Identifier: BSL-1.0
//
// TypePack was originally developed as part of OSVR-Core.
//
// Some files in this library incorporate code from "meta":
// Copyright Eric Niebler 2014-2015
//
// Use, modification and distribution is subject to the
// Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Project home: https://github.com/ericniebler/meta
//

#pragma once
#include <array>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

/// @brief A simple argument-pack-based metaprogramming library, inspired by
/// and based partially on https://ericniebler.github.io/meta
///
/// It includes an as-needed subset of the features of meta, modified as needed
/// to build on MSVC 2013, as well as additional functionality not found in
/// meta.
namespace typepack {
/// Apply an alias class
template <typename F, typename... Args>
using apply = typename F::template apply<Args...>;

/// Compose the Alias Classes \p Fs in the parameter pack \p Ts.
template <typename... Fs> struct compose {};

template <typename F0> struct compose<F0> {
    template <typename... Ts> using apply = typepack::apply<F0, Ts...>;
};

template <typename F0, typename... Fs> struct compose<F0, Fs...> {
    template <typename... Ts>
    using apply = typepack::apply<F0, typepack::apply<compose<Fs...>, Ts...>>;
};

namespace detail {
    struct list_base_ {};
} // namespace detail

/// @brief A wrapper for a template parameter pack of types.
///
/// Note that passing a single list<...> as the parameter to list<> will not
/// change the type (doesn't nest the lists), so this is safe. If you need
/// to ensure some argument is just a list, see typepack::coerce_list.
template <typename... Ts> struct list : detail::list_base_ {
    using type = list;
};
template <typename... Ts> struct list<list<Ts...>> : list<Ts...>::type {};

/// \cond
namespace detail {

    template <template <typename...> class, typename> struct defer_ {};

    template <template <typename...> class C, typename... Ts>
    struct defer_<C, list<Ts...>> {
        using type = C<Ts...>;
    };
} // namespace detail
/// \endcond

///////////////////////////////////////////////////////////////////////////////////////////
// defer
/// A wrapper that defers the instantiation of a template \p C with type
/// parameters \p Ts in
/// a \c lambda or \c let expression.
///
/// In the code below, the lambda would ideally be written as
/// `lambda<_a,_b,push_back<_a,_b>>`, however this fails since
/// `push_back` expects its first
/// argument to be a list, not a placeholder. Instead, we express it
/// using \c defer as
/// follows:
///
/// \code
/// template<typename List>
/// using reverse = reverse_fold<List, list<>, lambda<_a, _b,
/// defer<push_back, _a, _b>>>;
/// \endcode
template <template <typename...> class C, typename... Ts>
struct defer : detail::defer_<C, list<Ts...>> {};

/// @brief A convenience alias template to extract the nested `type` within
/// the supplied `T`.
///
/// The name was chosen to parallel how the traits in C++14 (like
/// `std::enable_if`) have been complemented by aliases ending in `_t` (like
/// `std::enable_if_t<COND>` being equivalent to `typename
/// std::enable_if<COND>::type`)
///
/// Note that the name is `t_`, unlike in `meta` where the semi-illegal name
/// `_t` is used. (Leading underscores are between risky and not permitted.)
template <typename T> using t_ = typename T::type;

/// Turn a class template or alias template \p C into a Alias Class.
/// @todo doesn't work if used more than once in a single translation unit
/// on MSVC2013?
template <template <typename...> class C> struct quote {
    // Indirection through defer here needed to avoid Core issue 1430
    // http://open-std.org/jtc1/sc22/wg21/docs/cwg_active.html#1430
    template <typename... Ts> using apply = t_<defer<C, Ts...>>;
};

/// Turn a trait \p C into a Alias Class.
template <template <typename...> class C>
using quote_trait = compose<quote<t_>, quote<C>>;
namespace detail {
    /// General/dummy case.
    template <typename F, typename List> struct apply_list_;
    template <typename F, typename... Args>
    struct apply_list_<F, list<Args...>> {
        // the simpler solution doesn't build with MSVC 2013
        template <typename...> using apply = typepack::apply<F, Args...>;
    };
} // namespace detail

/// Apply an alias class, exploding the list of args
template <typename F, typename Args>
using apply_list = apply<detail::apply_list_<F, Args>>;
/// @brief Alias template to simplify creating a boolean integral constant
template <bool V> using bool_ = std::integral_constant<bool, V>;
/// \cond
namespace detail {
    template <typename...> struct if_impl {};

    template <typename If>
    struct if_impl<If> : std::enable_if<If::type::value> {};

    template <typename If, typename Then>
    struct if_impl<If, Then> : std::enable_if<If::type::value, Then> {};

    template <typename If, typename Then, typename Else>
    struct if_impl<If, Then, Else>
        : std::conditional<If::type::value, Then, Else> {};
} // namespace detail
/// \endcond

/// Select one type or another depending on a compile-time Boolean integral
/// constant type.
template <typename... Args> using if_ = t_<detail::if_impl<Args...>>;

/// Select one type or another depending on a compile-time Boolean value.
template <bool If, typename... Args>
using if_c = t_<detail::if_impl<bool_<If>, Args...>>;

/// \cond
namespace detail {
    template <typename... Bools> struct or_impl;

    template <> struct or_impl<> : std::false_type {};

    template <typename Bool, typename... Bools>
    struct or_impl<Bool, Bools...>
        : if_c<Bool::type::value, std::true_type, or_impl<Bools...>> {};
} // namespace detail
/// \endcond

/// Logically or together all the integral constant-wrapped Boolean
/// parameters, \e with short-circuiting.
template <typename... Bools> using or_ = t_<detail::or_impl<Bools...>>;

namespace detail {

    /// The trait used to implement the alias typepack::transform
    template <typename List, typename Fun> struct transform_;
    template <typename... Ts, typename Fun>
    struct transform_<list<Ts...>, Fun> {
        /// required for MSVC2013 to avoid "there are no parameter packs
        /// available to expand"
        template <typename T> struct apply_workaround {
            using type = typepack::apply<Fun, T>;
        };
        using type = list<typename apply_workaround<Ts>::type...>;
    };
} // namespace detail

/// Given a list and an alias class, apply the alias class to each element
/// in the list and return the results in a list.
template <typename List, typename Fun>
using transform = t_<detail::transform_<List, Fun>>;

namespace detail {
    /// Bind the first argument of std::is_same
    template <typename T> struct is_ {
        template <typename Elt> using apply = std::is_same<T, Elt>;
    };

} // namespace detail

/// @brief Determines if type @p Needle is in the list @p Haystack - is an
/// alias for a type that inherits std::true_type or std::false_type.
template <typename Haystack, typename Needle>
using contains =
    apply_list<quote<or_>, transform<Haystack, detail::is_<Needle>>>;

/// @brief Alias template to simplify creating an integral constant of
/// size_t
template <std::size_t V> using size_t_ = std::integral_constant<std::size_t, V>;
namespace detail {
    template <typename Needle, std::size_t i, typename... Ts>
    struct find_first_impl;
    // Expand lists
    template <typename Needle, typename... Ts>
    struct find_first_impl<Needle, 0, list<Ts...>>
        : find_first_impl<Needle, 0, Ts...> {};
    // base case: at the head
    template <typename Needle, std::size_t i, typename... Ts>
    struct find_first_impl<Needle, i, Needle, Ts...> {
        using type = size_t_<i>;
    };
    // Recursive case
    template <typename Needle, std::size_t i, typename Head, typename... Ts>
    struct find_first_impl<Needle, i, Head, Ts...> {
        using type = t_<find_first_impl<Needle, i + 1, Ts...>>;
    };
    /// base case not found
    template <typename Needle, std::size_t i>
    struct find_first_impl<Needle, i> {};

} // namespace detail

/// @brief Returns the zero-based index of the first instance of @p Needle
/// in @p List. Will fail to compile if not found.
template <typename List, typename Needle>
using find_first = t_<detail::find_first_impl<Needle, 0, List>>;

// Forward declaration
template <typename Derived> class TypeKeyedBase;

namespace typekeyed_detail {
    /// traits class that MUST be specialized for each type-keyed
    /// container: should contain a `using type = ` declaration returning
    /// the value type corresponding to the given key type.
    template <typename Derived, typename Key> struct ValueTypeAtKeyTraits {};

    /// traits class that can be specialized for each type-keyed
    /// container: should contain a `using type = ` declaration providing
    /// the list of key types. Default implementation assumes member type
    /// name `key_types`
    template <typename Derived> struct KeyTypesTraits {
        using type = typename Derived::key_types;
    };

    /// Gets key types list for a given type-keyed container, using the
    /// traits class.
    template <typename Derived> using key_types = t_<KeyTypesTraits<Derived>>;

    /// Returns an integral_constant for whether a given key is valid for a
    /// given type-keyed container.
    template <typename Derived, typename Key>
    using valid_key = contains<key_types<Derived>, Key>;

    /// Gets the index of the key in a key list for a given type-keyed
    /// container
    template <typename Derived, typename Key>
    using index = find_first<key_types<Derived>, Key>;

    /// Gets the corresponding value type in a given type-keyed container
    /// for a given key type
    template <typename Derived, typename Key>
    using value_type_at_key = t_<ValueTypeAtKeyTraits<Derived, Key>>;

    /// Gets the corresponding reference to value type in a given type-keyed
    /// container for a given key type
    template <typename Derived, typename Key>
    using ref_type_at_key =
        std::add_lvalue_reference<value_type_at_key<Derived, Key>>;

    /// Gets the corresponding rvalue-reference to value type in a given
    /// type-keyed container for a given key type
    template <typename Derived, typename Key>
    using rref_type_at_key =
        std::add_rvalue_reference<value_type_at_key<Derived, Key>>;

    /// Gets the corresponding reference to constant value type in a given
    /// type-keyed container for a given key type
    template <typename Derived, typename Key>
    using cref_type_at_key = std::add_lvalue_reference<
        t_<std::add_const<value_type_at_key<Derived, Key>>>>;

    /// Class with static members performing the actual access. Specialize
    /// if  `std::get<index>()` doesn't work on your type-keyed structure's
    /// implementation or if it doesn't have a `nested_container()` member.
    ///
    ///
    /// Add <code>
    /// template <typename, typename>
    /// friend struct typekeyed_detail::ValueAccessor
    /// </code> to your
    /// and you can keep your `nested_container()` members private.
    template <typename Derived, typename Key> struct ValueAccessor {
        static_assert(valid_key<Derived, Key>::value,
                      "Key type not found in the list!");
        using reference = typename ref_type_at_key<Derived, Key>::type;
        using rvalue_reference = typename rref_type_at_key<Derived, Key>::type;
        using const_reference = typename cref_type_at_key<Derived, Key>::type;
        using Index = index<Derived, Key>;
        static reference get_reference(TypeKeyedBase<Derived> &c);
        static const_reference
        get_const_reference(TypeKeyedBase<Derived> const &c);
        static rvalue_reference
        get_rvalue_reference(TypeKeyedBase<Derived> &&c);
    };

    // forward declaration
    template <typename Key, typename Derived>
    typename ref_type_at_key<Derived, Key>::type get(TypeKeyedBase<Derived> &c);

    // forward declaration
    template <typename Key, typename Derived>
    typename rref_type_at_key<Derived, Key>::type
    rget(TypeKeyedBase<Derived> &&c);

    // forward declaration
    template <typename Key, typename Derived>
    typename cref_type_at_key<Derived, Key>::type
    cget(TypeKeyedBase<Derived> const &c);
} // namespace typekeyed_detail

/// CRTP base for type-keyed data types, providing a unified interface
/// (compile-time polymorphism) and shared functionality. Methods and
/// related free functions work on all type-keyed containers derived from
/// this base class.
template <typename Derived> class TypeKeyedBase {
  public:
    using DerivedType = Derived;

    /// Get a reference to a value in the container, keyed by the provided
    /// type (either explicit or deduced from the argument).
    template <typename Key>
    typename typekeyed_detail::ref_type_at_key<Derived, Key>::type
    get(Key const * = nullptr) {
        return typekeyed_detail::get<Key>(*this);
    }

    /// Get a reference to a constant value in the container, keyed by the
    /// provided type (either explicit or deduced from the argument).
    template <typename Key>
    typename typekeyed_detail::cref_type_at_key<Derived, Key>::type
    get(Key const * = nullptr) const {
        return typekeyed_detail::cget<Key>(*this);
    }

  private:
    /// CRTP: access derived class.
    DerivedType &derived() { return *static_cast<DerivedType *>(this); }

    /// CRTP: const-access derived class.
    DerivedType const &derived() const {
        return *static_cast<DerivedType const *>(this);
    }

    /// CRTP: const-access derived class.
    DerivedType const &const_derived() const {
        return *static_cast<DerivedType const *>(this);
    }

    // befriend the only consumer of our `derived()` methods.
    template <typename, typename> friend struct typekeyed_detail::ValueAccessor;
};

namespace typekeyed_detail {
    template <typename Derived, typename Key>
    inline typename ref_type_at_key<Derived, Key>::type
    ValueAccessor<Derived, Key>::get_reference(TypeKeyedBase<Derived> &c) {
        return std::get<Index::value>(c.derived().nested_container());
    }
    template <typename Derived, typename Key>
    inline typename cref_type_at_key<Derived, Key>::type
    ValueAccessor<Derived, Key>::get_const_reference(
        TypeKeyedBase<Derived> const &c) {
        return std::get<Index::value>(c.derived().nested_container());
    }
    template <typename Derived, typename Key>
    inline typename rref_type_at_key<Derived, Key>::type
    ValueAccessor<Derived, Key>::get_rvalue_reference(
        TypeKeyedBase<Derived> &&c) {
        return std::forward<rvalue_reference>(
            std::get<Index::value>(c.derived().nested_container()));
    }
    /// Gets a reference to a value in a type-keyed container using the
    /// specified key type.
    template <typename Key, typename Derived>
    inline typename ref_type_at_key<Derived, Key>::type
    get(TypeKeyedBase<Derived> &c) {
        static_assert(valid_key<Derived, Key>::value,
                      "Key type not found in the list!");
        return ValueAccessor<Derived, Key>::get_reference(c);
    }
    /// Gets a rvalue-reference to a value in a type-keyed container using
    /// the specified key type.
    template <typename Key, typename Derived>
    inline typename rref_type_at_key<Derived, Key>::type
    rget(TypeKeyedBase<Derived> &&c) {
        static_assert(valid_key<Derived, Key>::value,
                      "Key type not found in the list!");
        return std::forward<rref_type_at_key<Derived, Key>>(
            ValueAccessor<Derived, Key>::get_rvalue_reference(
                std::forward<TypeKeyedBase<Derived> &&>(c)));
    }

    /// Gets an lvalue-reference to a constant value in a type-keyed
    /// container using the specified key type.
    template <typename Key, typename Derived>
    inline typename cref_type_at_key<Derived, Key>::type
    cget(const TypeKeyedBase<Derived> &c) {
        static_assert(valid_key<Derived, Key>::value,
                      "Key type not found in the list!");
        return ValueAccessor<Derived, Key>::get_const_reference(c);
    }

} // namespace typekeyed_detail

/// Gets a reference to a value in a type-keyed container using the
/// specified key type.
///
/// @relates TypeKeyedBase
template <typename Key, typename Derived>
inline typename typekeyed_detail::ref_type_at_key<Derived, Key>::type
get(TypeKeyedBase<Derived> &c) {
    return typekeyed_detail::get<Key>(c);
}

/// Gets an rvalue-reference to a value in a type-keyed container using the
/// specified key type.
///
/// @relates TypeKeyedBase
template <typename Key, typename Derived>
inline typename typekeyed_detail::rref_type_at_key<Derived, Key>::type
rget(TypeKeyedBase<Derived> &&c) {
    return std::forward<typekeyed_detail::rref_type_at_key<Derived, Key>>(
        typekeyed_detail::rget<Key>(c));
}
/// Gets a reference to a constant value in a type-keyed container using the
/// specified key type.
///
/// @relates TypeKeyedBase
///
/// @todo figure out why MSVC can't perform overload resolution with a
/// non-const argument if this is named `get`
template <typename Key, typename Derived>
inline typename typekeyed_detail::cref_type_at_key<Derived, Key>::type
cget(TypeKeyedBase<Derived> const &c) {
    return typekeyed_detail::cget<Key>(c);
}
/// @brief Provides a data structure where a value of heterogeneous data
/// types may be stored at runtime for each of the "key" types in a
/// list. The runtime data type stored is computed by an alias class.
///
/// Vaguely replaces the functionality of a boost::fusion::map.
///
/// Values can be accessed with the nonmember get() free common to
/// type-keyed containers.
///
/// Element access performance is equal to `get()` on a std::tuple
/// (should be constant)
template <typename KeyList, typename ComputeValueTypes>
class TypeKeyedTuple
    : public TypeKeyedBase<TypeKeyedTuple<KeyList, ComputeValueTypes>> {
    using value_types = transform<KeyList, ComputeValueTypes>;

  public:
    using key_types = KeyList;

    using container_type = apply_list<quote<std::tuple>, value_types>;

  private:
    template <typename, typename> friend struct typekeyed_detail::ValueAccessor;

    /// Internal method/implementation detail, do not use in consuming code!
    container_type &nested_container() { return container_; }
    /// Internal method/implementation detail, do not use in consuming code!
    container_type const &nested_container() const { return container_; }

  private:
    container_type container_;
};

// Required traits
namespace typekeyed_detail {
    template <typename KeyList, typename ComputeValueTypes, typename Key>
    struct ValueTypeAtKeyTraits<TypeKeyedTuple<KeyList, ComputeValueTypes>,
                                Key> {
        using type = apply<ComputeValueTypes, Key>;
    };
} // namespace typekeyed_detail

namespace detail {
    template <typename T> struct push_back_impl {
        template <typename... Ts> using apply = list<Ts..., T>;
    };
} // namespace detail
template <typename List, typename T>
using push_back = apply_list<detail::push_back_impl<T>, List>;
/// \cond
namespace detail {
    template <typename... Lists> struct concat_ {};

    template <> struct concat_<> { using type = list<>; };

    template <typename... List1> struct concat_<list<List1...>> {
        using type = list<List1...>;
    };

    template <typename... List1, typename... List2>
    struct concat_<list<List1...>, list<List2...>> {
        using type = list<List1..., List2...>;
    };

    template <typename... List1, typename... List2, typename... List3>
    struct concat_<list<List1...>, list<List2...>, list<List3...>> {
        using type = list<List1..., List2..., List3...>;
    };

    template <typename... List1, typename... List2, typename... List3,
              typename... Rest>
    struct concat_<list<List1...>, list<List2...>, list<List3...>, Rest...>
        : concat_<list<List1..., List2..., List3...>, Rest...> {};
} // namespace detail
/// \endcond

/// Concatenates several lists into a single list.
/// \pre The parameters must all be instantiations of \c typepack::list.
/// \par Complexity
/// \f$ O(L) \f$ where \f$ L \f$ is the number of lists in the list of
/// lists.
template <typename... Lists> using concat = t_<detail::concat_<Lists...>>;

namespace detail {
    template <typename T> struct push_front_impl {
        template <typename... Ts> using apply = list<T, Ts...>;
    };
} // namespace detail

template <typename List, typename T>
using push_front = apply_list<detail::push_front_impl<T>, List>;

/// Logical not on a single boolean.
template <typename Bool> using not_ = bool_<!Bool::value>;

/// @brief Will turn whatever is passed into it into the simplest list.
template <typename... Ts> using coerce_list = t_<list<Ts...>>;

namespace detail {
    template <typename... Ts> struct size;

    // The following fails with clang due to a bug.
    // <https://llvm.org/bugs/show_bug.cgi?id=14858>
    // template <typename... Ts> using size_impl =
    // size_t_<sizeof...(Ts)>;
    // template <typename... Ts>
    // struct size<list<Ts...>> : size_impl<Ts...> {};
    template <typename... Ts>
    struct size<list<Ts...>> : size_t_<sizeof...(Ts)> {};
} // namespace detail

/// @brief Get the size of a list (number of elements.)
template <typename... Ts> using size = detail::size<coerce_list<Ts...>>;

namespace detail {
    /// General/dummy case.
    template <typename... List> struct split_list_ {
        using head = void;
        using tail = list<>;
    };

    /// Unwrap type pack.
    template <typename... List>
    struct split_list_<list<List...>> : split_list_<List...> {};

    /// Primary case
    template <typename Head, typename... Tail>
    struct split_list_<Head, Tail...> {
        using head = Head;
        using tail = list<Tail...>;
    };
} // namespace detail

/// Get the first element of a list.
template <typename... List>
using head = typename detail::split_list_<List...>::head;

/// Get the list without its first element
template <typename... List>
using tail = typename detail::split_list_<List...>::tail;
namespace detail {
    /// Forward declaration of the implementation struct - because we can't
    /// generate code for a branch when the tail is empty.
    template <typename F, typename Head, typename Tail, bool EmptyTail>
    struct for_each_type_impl;

    /// Implementation function that handles the recursion/invocation of the
    /// right implementation struct.
    template <typename F, typename List, typename... Args>
    inline void for_each_type_(F &&f, Args &&... args) {
        using H = typepack::head<List>;
        using T = typepack::tail<List>;
        using Sz = typepack::size<T>;
        static const auto empty = (Sz::value == 0);
        using Impl = for_each_type_impl<F, H, T, empty>;
        Impl::apply(std::forward<F>(f), std::forward<Args>(args)...);
    }

    /// Specialization of struct for when the tail is empty: just calls the
    /// function
    template <typename F, typename Head, typename Tail>
    struct for_each_type_impl<F, Head, Tail, true> {
        template <typename... Args> static void apply(F &&f, Args &&... args) {
            f(Head{}, std::forward<Args>(args)...);
        }
    };

    /// Specialization of struct for when the tail is non-empty: calls the
    /// function, then calls the implementation/recursion method above with
    /// the tail.
    template <typename F, typename Head, typename Tail>
    struct for_each_type_impl<F, Head, Tail, false> {
        template <typename... Args> static void apply(F &&f, Args &&... args) {
            f(Head{}, std::forward<Args>(args)...);
            for_each_type_<F, Tail>(std::forward<F>(f),
                                    std::forward<Args>(args)...);
        }
    };
} // namespace detail

/// Run-time operation on a type list: given a function object, a type list,
/// and optional arguments to be forwarded to the function call, construct
/// each type in the type list in turn and call the function-call-operator
/// of your functor with it (and your optional additional arguments).
template <typename List, typename F, typename... Args>
inline void for_each_type(F &&f, Args &&... args) {
    detail::for_each_type_<F, List>(std::forward<F>(f),
                                    std::forward<Args>(args)...);
}

/// \cond
namespace detail {
    template <typename... Bools> struct and_impl;

    template <> struct and_impl<> : std::true_type {};

    template <typename Bool, typename... Bools>
    struct and_impl<Bool, Bools...>
        : if_<bool_<!Bool::type::value>, std::false_type, and_impl<Bools...>> {
    };

} // namespace detail
/// \endcond

/// Logically and together all the integral constant-wrapped Boolean
/// parameters, \e with short-circuiting.
template <typename... Bools> using and_ = t_<detail::and_impl<Bools...>>;

/// @brief A class that uses types as an index into a container with
/// uniform-typed contents, somewhat like a map except all elements are
/// default-constructed rather than having an optional "not set" status.
/// (You may emulate this by providing a specialization of
/// boost::optional<> as your value type.)
///
/// Values can be accessed just as all other type-keyed containers.
///
/// Runtime performance of element access is constant (equal to array
/// element access with a constant index)
template <typename KeyList, typename ValueType>
class TypeKeyedMap : public TypeKeyedBase<TypeKeyedMap<KeyList, ValueType>> {
    using size_constant = size<KeyList>;

  public:
    using key_types = KeyList;
    using value_type = ValueType;

  private:
    template <typename, typename> friend struct typekeyed_detail::ValueAccessor;
    using container_type = std::array<value_type, size_constant::value>;
    /// Internal method/implementation detail, do not use in consuming code!
    container_type &nested_container() { return container_; }
    /// Internal method/implementation detail, do not use in consuming code!
    container_type const &nested_container() const { return container_; }

  private:
    container_type container_;
};

// Required traits
namespace typekeyed_detail {
    template <typename KeyList, typename ValueType, typename Key>
    struct ValueTypeAtKeyTraits<TypeKeyedMap<KeyList, ValueType>, Key> {
        using type = ValueType;
    };

} // namespace typekeyed_detail

namespace detail {
    // Fold: Forward declaration of general form
    template <typename List, typename State, typename Fun> struct fold_;

    // Fold: Recurse
    template <typename List, typename State, typename Fun>
    struct fold_ : fold_<tail<List>, t_<apply<Fun, State, head<List>>>, Fun> {};

    // Fold: base case
    template <typename State, typename Fun> struct fold_<list<>, State, Fun> {
        using type = State;
    };

} // namespace detail

/// @brief Fold the list (right) with the given alias class and initial
/// state.
template <typename List, typename State, typename Fun>
using fold = t_<detail::fold_<List, State, Fun>>;

/// @brief A Alias Class that always returns \p T.
template <typename T> struct always {
  private:
    // Redirect through a class template for compilers that have not
    // yet implemented CWG 1558:
    // <http://www.open-std.org/jtc1/sc22/wg21/docs/cwg_defects.html#1558>
    template <typename...> struct impl { using type = T; };

  public:
    template <typename... Ts> using apply = t_<impl<Ts...>>;
};

/// An alias for `void`.
template <typename... Ts> using void_ = apply<always<void>, Ts...>;

/// \cond
namespace detail {
    template <typename, typename = void> struct has_type_ {
        using type = std::false_type;
    };

    template <typename T> struct has_type_<T, void_<typename T::type>> {
        using type = std::true_type;
    };

} // namespace detail
/// \endcond

/// An alias for `std::true_type` if `T::type` exists and names a type;
/// otherwise, it's an alias for `std::false_type`.
template <typename T> using has_type = t_<detail::has_type_<T>>;
} // namespace typepack
