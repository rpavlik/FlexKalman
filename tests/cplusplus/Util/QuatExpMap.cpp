/** @file
    @brief Implementation

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
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

// Internal Includes
#include "CatchEigen.h"
#include "FlexKalman/EigenQuatExponentialMap.h"

// Library/third-party includes
#ifdef HAVE_QUATLIB
#include "quat.h"
#endif // HAVE_QUATLIB
#include <catch2/catch.hpp>

// Standard includes
#include <array>

#ifndef HAVE_QUATLIB
#define Q_X 0
#define Q_Y 1
#define Q_Z 2
#define Q_W 3
#endif // !HAVE_QUATLIB

using flexkalman::util::quat_exp;
using flexkalman::util::quat_ln;

static const double SMALL = 0.1;
static const double SMALLER = 1.0e-5;

// Make an equality comparison for quaternions, for the sake of Catch2.

namespace Eigen {
static inline bool operator==(Quaterniond const &lhs, Quaterniond const &rhs) {
    return lhs.coeffs() == rhs.coeffs();
}
} // namespace Eigen

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

/// @name Quatlib interaction utilities
/// @{
/// Container for q_type that's C++-safe to pass around and such. To pass to
/// quatlib functions, use the `.data()` member function.
using QuatArray = std::array<double, 4>;

/// Convert Eigen vector to a quatlib (pure: w = 0) quaternion, wrapped in an
/// std::array.
static inline QuatArray toQuatlib(Vector3d const &vec) {
    QuatArray ret;
    ret[Q_W] = 0;
    ret[Q_X] = vec.x();
    ret[Q_Y] = vec.y();
    ret[Q_Z] = vec.z();
    return ret;
}

/// Convert Eigen quat to a quatlib quaternion, wrapped in an std::array.
static inline QuatArray toQuatlib(Quaterniond const &q) {
    QuatArray ret;
    ret[Q_W] = q.w();
    ret[Q_X] = q.x();
    ret[Q_Y] = q.y();
    ret[Q_Z] = q.z();
    return ret;
}

/// Takes a quatlib quaternion wrapped in an array and converts it to an
/// Eigen::Quaterniond, no questions asked.
static inline Quaterniond quatFromQuatlib(QuatArray const &arr) {
    return Quaterniond(arr[Q_W], arr[Q_X], arr[Q_Y], arr[Q_Z]);
}
/// Takes a quatlib quaternion wrapped in an array and converts it to an
/// Eigen::Vector3d, no questions asked - assumes it's a pure quaternion (w=0)
/// or that you just want the vector part.
static inline Vector3d vecFromQuatlib(QuatArray const &arr) {
    return Vector3d(arr[Q_X], arr[Q_Y], arr[Q_Z]);
}
/// @}

/// Creates a quaternion from angle+axis or identity,
/// and stores it along with a human-readable description.
class QuatCreator {
  public:
    explicit QuatCreator(QuatArray &&arr, std::string &&input)
        : m_coeffs(arr), m_input(std::move(input)) {}

    static QuatCreator Identity() {
        return QuatCreator(toQuatlib(Eigen::Quaterniond::Identity()),
                           "Identity");
    }
    static QuatCreator AngleAxis(double angle, Eigen::Vector3d const &axis) {
        return QuatCreator(
            toQuatlib(Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis))),
            formatAngleAxis(angle, axis));
    }

    Eigen::Quaterniond get() const { return quatFromQuatlib(m_coeffs); }

    std::string const &getDescription() const { return m_input; }

  private:
    QuatArray m_coeffs;
    std::string m_input;
    static std::string formatAngleAxis(double angle,
                                       Eigen::Vector3d const &axis) {
        std::ostringstream os;
        os << "Angle " << angle << ", Axis " << axis.transpose();
        return os.str();
    }
};

static inline ::std::ostream &operator<<(::std::ostream &os,
                                         QuatCreator const &q) {
    os << q.getDescription();
    return os;
}

using QuatVecPair = std::pair<QuatCreator, Eigen::Vector3d>;

static inline QuatVecPair makePairFromAngleAxis(double angle,
                                                Eigen::Vector3d const &axis) {
    return std::make_pair(QuatCreator::AngleAxis(angle, axis),
                          (angle * axis * 0.5).eval());
}

static inline ::std::ostream &operator<<(::std::ostream &os,
                                         QuatVecPair const &q) {
    os << q.first << " (quat-vec pair, vec " << q.second.transpose() << ")";
    return os;
}

static const Vector3d Vec3dZero = Vector3d::Zero();

/* Tests that take a unit quat as input */
static auto BasicQuats = {
    QuatCreator::Identity(),
    QuatCreator::AngleAxis(EIGEN_PI / 2, Vector3d::UnitX()),
    QuatCreator::AngleAxis(EIGEN_PI / 2, Vector3d::UnitY()),
    QuatCreator::AngleAxis(EIGEN_PI / 2, Vector3d::UnitZ()),
    QuatCreator::AngleAxis(-EIGEN_PI / 2, Vector3d::UnitX()),
    QuatCreator::AngleAxis(-EIGEN_PI / 2, Vector3d::UnitY()),
    QuatCreator::AngleAxis(-EIGEN_PI / 2, Vector3d::UnitZ())};

static auto SmallQuats = {QuatCreator::AngleAxis(SMALL, Vector3d::UnitX()),
                          QuatCreator::AngleAxis(SMALL, Vector3d::UnitY()),
                          QuatCreator::AngleAxis(SMALL, Vector3d::UnitZ()),
                          QuatCreator::AngleAxis(SMALLER, Vector3d::UnitX()),
                          QuatCreator::AngleAxis(SMALLER, Vector3d::UnitY()),
                          QuatCreator::AngleAxis(SMALLER, Vector3d::UnitZ())};
static auto SmallNegativeQuats = {
    QuatCreator::AngleAxis(-SMALL, Vector3d::UnitX()),
    QuatCreator::AngleAxis(-SMALL, Vector3d::UnitY()),
    QuatCreator::AngleAxis(-SMALL, Vector3d::UnitZ()),
    QuatCreator::AngleAxis(-SMALLER, Vector3d::UnitX()),
    QuatCreator::AngleAxis(-SMALLER, Vector3d::UnitY()),
    QuatCreator::AngleAxis(-SMALLER, Vector3d::UnitZ())};

#if 0
QuatCreator::AngleAxis(EIGEN_PI, Vector3d::UnitX()),
QuatCreator::AngleAxis(EIGEN_PI, Vector3d::UnitY()),
QuatCreator::AngleAxis(EIGEN_PI, Vector3d::UnitZ()),
QuatCreator::AngleAxis(3 * EIGEN_PI / 2, Vector3d::UnitX()),
QuatCreator::AngleAxis(3 * EIGEN_PI / 2, Vector3d::UnitY()),
QuatCreator::AngleAxis(3 * EIGEN_PI / 2, Vector3d::UnitZ()),
#endif

TEST_CASE("UnitQuatInput") {
    const auto doTests = [](QuatCreator const &qCreator) {
        CAPTURE(qCreator);
        Quaterniond q = qCreator.get();
        CAPTURE(q);
        SECTION("Basic run ln") {
            REQUIRE_NOTHROW(quat_ln(q));
            const Vector3d ln_q = quat_ln(q);
            const bool isIdentityQuat = q.vec().norm() == 0;
            CAPTURE(isIdentityQuat);
            if (isIdentityQuat) {
                REQUIRE(ln_q == Vec3dZero);
            } else {
                REQUIRE_FALSE(ln_q == Vec3dZero);
            }
            SECTION("Round trip") {

                const Quaterniond exp_ln_q = quat_exp(ln_q);

                REQUIRE(q == exp_ln_q);
            }
        }
#ifdef HAVE_QUATLIB
        SECTION("Quatlib roundtrip (exp(ln(q))) as ground truth") {
            QuatArray quatlib_q = toQuatlib(q);
            q_log(quatlib_q.data(), quatlib_q.data());
            q_exp(quatlib_q.data(), quatlib_q.data());
            REQUIRE(ApproxVec(q.coeffs()) ==
                    quatFromQuatlib(quatlib_q).coeffs());
        }
#endif // HAVE_QUATLIB
    };
    SECTION("Basic quats") {
        auto quatCreator = GENERATE(values(BasicQuats));
        doTests(quatCreator);
    }
    SECTION("Small quats") {
        auto quatCreator = GENERATE(values(SmallQuats));
        doTests(quatCreator);
    }
    SECTION("Small negative quats") {
        auto quatCreator = GENERATE(values(SmallNegativeQuats));
        doTests(quatCreator);
    }
}

/* Tests that take a rotation vector as input */
static const std::initializer_list<Vector3d> BasicVecs = {
    Vector3d::Zero(),
    Vector3d(EIGEN_PI / 2, 0, 0),
    Vector3d(0, EIGEN_PI / 2, 0),
    Vector3d(0, 0, EIGEN_PI / 2),
    Vector3d(-EIGEN_PI / 2, 0, 0),
    Vector3d(0, -EIGEN_PI / 2, 0),
    Vector3d(0, 0, -EIGEN_PI / 2)};
static const std::initializer_list<Vector3d> SmallVecs = {
    Vector3d(SMALL, 0, 0),   Vector3d(0, SMALL, 0),   Vector3d(0, 0, SMALL),
    Vector3d(SMALLER, 0, 0), Vector3d(0, SMALLER, 0), Vector3d(0, 0, SMALLER)};

static const std::initializer_list<Vector3d> SmallNegativeVecs = {
    Vector3d(-SMALL, 0, 0),   Vector3d(0, -SMALL, 0),
    Vector3d(0, 0, -SMALL),   Vector3d(-SMALLER, 0, 0),
    Vector3d(0, -SMALLER, 0), Vector3d(0, 0, -SMALLER)};

TEST_CASE("ExpMapVecInput") {
    const auto doTests = [](Vector3d const &v) {
        CAPTURE(v);
        SECTION("BasicRunExp") {
            REQUIRE_NOTHROW(quat_exp(v));
            const Quaterniond exp_v = quat_exp(v);

            const bool isNullRotation = (v == Vector3d::Zero());
            CAPTURE(isNullRotation);

            if (isNullRotation) {
                REQUIRE(exp_v == Quaterniond::Identity());
            } else {
                REQUIRE_FALSE(exp_v == Quaterniond::Identity());
            }

            SECTION("Round-trip") {
                Vector3d ln_exp_v = quat_ln(exp_v);
                REQUIRE(ln_exp_v == ApproxVec(v));
            }
        }

#ifdef HAVE_QUATLIB
        SECTION("Quatlib roundtrip (ln(exp(v))) as ground truth") {
            QuatArray quatlib_q = toQuatlib(v);
            q_exp(quatlib_q.data(), quatlib_q.data());
            q_log(quatlib_q.data(), quatlib_q.data());
            REQUIRE(ApproxVec(v) == vecFromQuatlib(quatlib_q));
        }
#endif // HAVE_QUATLIB
    };
    SECTION("BasicVecs") {
        Vector3d v = GENERATE(values(BasicVecs));
        doTests(v);
    }
    SECTION("SmallVecs") {
        Vector3d v = GENERATE(values(SmallVecs));
        doTests(v);
    }
    SECTION("SmallNegativeVecs") {
        Vector3d v = GENERATE(values(SmallNegativeVecs));
        doTests(v);
    }
}

TEST_CASE("SimpleEquivalencies-Ln") {
    REQUIRE(Vec3dZero == quat_ln(Quaterniond::Identity()));
}

TEST_CASE("SimpleEquivalencies-Exp") {
    REQUIRE(Quaterniond::Identity() == quat_exp(Vec3dZero));
}

/* Tests that take a pair of equivalent quaternion and vector as input */
static auto HalfPiMultiples = {
    makePairFromAngleAxis(EIGEN_PI / 2, Vector3d::UnitX()),
    makePairFromAngleAxis(EIGEN_PI / 2, Vector3d::UnitY()),
    makePairFromAngleAxis(EIGEN_PI / 2, Vector3d::UnitZ()),
    makePairFromAngleAxis(-EIGEN_PI / 2, Vector3d::UnitX()),
    makePairFromAngleAxis(-EIGEN_PI / 2, Vector3d::UnitY()),
    makePairFromAngleAxis(-EIGEN_PI / 2, Vector3d::UnitZ())};

static auto SmallEquivalentValues = {
    makePairFromAngleAxis(SMALL, Vector3d::UnitX()),
    makePairFromAngleAxis(SMALL, Vector3d::UnitY()),
    makePairFromAngleAxis(SMALL, Vector3d::UnitZ()),
    makePairFromAngleAxis(SMALLER, Vector3d::UnitX()),
    makePairFromAngleAxis(SMALLER, Vector3d::UnitY()),
    makePairFromAngleAxis(SMALLER, Vector3d::UnitZ())};

static auto SmallNegativeEquivalentValues = {
    makePairFromAngleAxis(-SMALL, Vector3d::UnitX()),
    makePairFromAngleAxis(-SMALL, Vector3d::UnitY()),
    makePairFromAngleAxis(-SMALL, Vector3d::UnitZ()),
    makePairFromAngleAxis(-SMALLER, Vector3d::UnitX()),
    makePairFromAngleAxis(-SMALLER, Vector3d::UnitY()),
    makePairFromAngleAxis(-SMALLER, Vector3d::UnitZ())};

TEST_CASE("EquivalentInput") {
    const auto doTests = [](QuatCreator const &qCreator, Vector3d const &v) {
        CAPTURE(v);
        CAPTURE(qCreator);
        Quaterniond q = qCreator.get();
        CAPTURE(q);
        Vector3d ln_q = quat_ln(q);
        Quaterniond exp_v = quat_exp(v);

        SECTION("Ln") { REQUIRE(ln_q == ApproxVec(v)); }

        SECTION("Exp") { REQUIRE(exp_v == ApproxQuat(q)); }

#ifdef HAVE_QUATLIB
        SECTION("Compare ln with quatlib") {
            QuatArray quatlib_q = toQuatlib(q);
            q_log(quatlib_q.data(), quatlib_q.data());
            REQUIRE(vecFromQuatlib(quatlib_q) == ln_q);
        }
        SECTION("Compare exp with quatlib") {
            QuatArray quatlib_q = toQuatlib(v);
            q_exp(quatlib_q.data(), quatlib_q.data());
            q_normalize(quatlib_q.data(), quatlib_q.data());

            REQUIRE(quatFromQuatlib(quatlib_q) == exp_v);
        }
#endif // HAVE_QUATLIB
    };
    SECTION("HalfPiMultiples") {
        QuatVecPair qvp = GENERATE(values(HalfPiMultiples));
        doTests(qvp.first, qvp.second);
    }
    SECTION("SmallEquivalentValues") {
        QuatVecPair qvp = GENERATE(values(SmallEquivalentValues));
        doTests(qvp.first, qvp.second);
    }
    SECTION("SmallNegativeVecs") {
        QuatVecPair qvp = GENERATE(values(SmallNegativeEquivalentValues));
        doTests(qvp.first, qvp.second);
    }
}
