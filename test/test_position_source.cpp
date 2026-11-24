#include "pixi_kuka_ros2/position_source.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/parsers/urdf.hpp>

namespace pixi_kuka_ros2 {
namespace {

constexpr double kDt = 0.002;  // 500 Hz, the cell's update rate
const std::vector<std::string> kJoints = {"lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4",
                                          "lbr_A5", "lbr_A6", "lbr_A7"};

std::string read_urdf() {
  std::ifstream file(PIXI_KUKA_TEST_URDF);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

const std::string &urdf() {
  static const std::string text = read_urdf();
  return text;
}

Eigen::VectorXd nominal() {
  Eigen::VectorXd q(7);
  q << 0.1, 0.5, -0.1, -1.2, 0.1, 0.8, 0.2;
  return q;
}

void urdf_limits(Eigen::VectorXd *lower, Eigen::VectorXd *upper) {
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf(), model);
  lower->resize(7);
  upper->resize(7);
  for (int index = 0; index < 7; ++index) {
    const auto &joint = model.joints[model.getJointId(kJoints[index])];
    (*lower)[index] = model.lowerPositionLimit[joint.idx_q()];
    (*upper)[index] = model.upperPositionLimit[joint.idx_q()];
  }
}

// The gap the expectations below are written for: 0.05 rad on A1 to A3, down to 0.015 on A7.
PositionSourceParams ik_params() {
  PositionSourceParams params;
  params.sunrise_joint_stiffness.assign(7, 200.0);
  params.max_spring_torque_nm = {10.0, 10.0, 10.0, 8.0, 5.0, 5.0, 3.0};
  return params;
}

PositionSourceParams unclamped(PositionSourceParams params) {
  params.max_spring_torque_nm.assign(7, 1e6);
  params.max_joint_step_rad.assign(7, 1e6);
  return params;
}

PositionSource make_source(const PositionSourceParams &params) {
  PositionSource source;
  std::string error;
  EXPECT_TRUE(source.load_model(urdf(), kJoints, &error)) << error;
  EXPECT_TRUE(source.configure(kJoints.size(), params, &error)) << error;
  EXPECT_TRUE(source.kinematics_ready()) << source.kinematics_error();
  return source;
}

PoseTarget fk(PositionSource &source, const Eigen::VectorXd &q) {
  PoseTarget pose;
  EXPECT_TRUE(source.forward_kinematics(q, &pose));
  return pose;
}

double position_error(PositionSource &source, const Eigen::VectorXd &q, const PoseTarget &goal) {
  return (fk(source, q).position - goal.position).norm();
}

double rotation_error(PositionSource &source, const Eigen::VectorXd &q, const PoseTarget &goal) {
  return fk(source, q).orientation.angularDistance(goal.orientation);
}

Eigen::VectorXd gap_of(const PositionSourceParams &params) {
  Eigen::VectorXd gap(7);
  for (int index = 0; index < 7; ++index) {
    gap[index] = params.max_spring_torque_nm[index] / params.sunrise_joint_stiffness[index];
  }
  return gap;
}

bool bit_identical(const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

TEST(PositionSourceParams, InvalidValuesFailConfigure) {
  PositionSource source;
  std::string error;
  ASSERT_TRUE(source.configure(7, ik_params(), &error)) << error;

  auto expect_rejected = [&](PositionSourceParams params, const char *what) {
    std::string reason;
    EXPECT_FALSE(source.configure(7, params, &reason)) << what;
    EXPECT_FALSE(reason.empty()) << what;
  };
  auto params = ik_params();
  params.max_joint_step_rad.assign(6, 0.004);
  expect_rejected(params, "6 steps for 7 joints");
  params = ik_params();
  params.sunrise_joint_stiffness[2] = 0.0;
  expect_rejected(params, "zero stiffness");
  params = ik_params();
  params.max_spring_torque_nm[0] = -1.0;
  expect_rejected(params, "negative torque");
  params = ik_params();
  params.max_joint_step_rad[3] = std::numeric_limits<double>::quiet_NaN();
  expect_rejected(params, "NaN step");
  params = ik_params();
  params.target_filter = 1.0;
  expect_rejected(params, "filter 1");
  params = ik_params();
  params.target_timeout_s = 0.0;
  expect_rejected(params, "timeout 0");

  // A margin is checked against the limits, so only once a model is loaded.
  ASSERT_TRUE(source.load_model(urdf(), kJoints, &error)) << error;
  params = ik_params();
  params.joint_limit_margin_rad = 2.5;
  expect_rejected(params, "margin wider than A2's range");
}

TEST(PositionSourceModel, RejectsJointsTheUrdfLacks) {
  PositionSource source;
  std::string error;
  EXPECT_FALSE(source.load_model(urdf(), {"lbr_A1", "lbr_A9"}, &error));
  EXPECT_FALSE(source.load_model(urdf(), {"lbr_A1", "lbr_A1"}, &error));
  EXPECT_FALSE(source.load_model("<robot", kJoints, &error));
  EXPECT_FALSE(source.model_loaded());
}

TEST(PositionSourceIk, ConvergesToReachableTargets) {
  std::mt19937 random(3);
  std::uniform_real_distribution<double> offset(-0.4, 0.4);
  for (const double nullspace : {0.0, 0.05}) {
    auto params = unclamped(ik_params());
    params.target_filter = 0.0;
    params.nullspace_gain = nullspace;
    PositionSource source = make_source(params);
    for (int trial = 0; trial < 20; ++trial) {
      Eigen::VectorXd goal = nominal();
      for (int index = 0; index < 7; ++index) goal[index] += offset(random);
      const PoseTarget target = fk(source, goal);
      const Eigen::VectorXd q_meas = nominal();
      source.activate(q_meas);
      for (int update = 0; update < 200; ++update) {
        source.update(q_meas, kDt, &target);
        ASSERT_FALSE(source.stale());
      }
      EXPECT_TRUE(bit_identical(source.q_out(), source.q_ik()));
      // With a posture pull the damped projector leaves a residual of order lambda^2 k |v|.
      const double tolerance = nullspace > 0.0 ? 1e-4 : 1e-9;
      EXPECT_LT(position_error(source, source.q_ik(), target), tolerance)
          << "trial " << trial << " nullspace " << nullspace;
      EXPECT_LT(rotation_error(source, source.q_ik(), target), tolerance)
          << "trial " << trial << " nullspace " << nullspace;
    }
  }
}

TEST(PositionSourceIk, ConvergesThroughTheTargetFilter) {
  PositionSource source = make_source(unclamped(ik_params()));
  const Eigen::VectorXd q_meas = nominal();
  source.activate(q_meas);
  PoseTarget target = fk(source, nominal());
  target.position += Eigen::Vector3d(0.03, -0.02, 0.01);
  target.orientation = target.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  source.update(q_meas, kDt, &target);
  PoseTarget moved = target;
  moved.position.z() += 0.02;
  double first_error = 0.0;
  for (int update = 0; update < 1000; ++update) {
    source.update(q_meas, kDt, &moved);
    if (update == 0) first_error = position_error(source, source.q_ik(), moved);
  }
  EXPECT_GT(first_error, 0.01);  // the filter: not there after one update
  EXPECT_LT(position_error(source, source.q_ik(), moved), 1e-4);
  EXPECT_LT(rotation_error(source, source.q_ik(), moved), 1e-4);
}

TEST(PositionSourceIk, GapClampHoldsForAFarTarget) {
  const auto params = ik_params();
  const Eigen::VectorXd gap = gap_of(params);
  EXPECT_NEAR(gap[0], 0.05, 1e-15);
  EXPECT_NEAR(gap[6], 0.015, 1e-15);
  PositionSource source = make_source(params);
  const Eigen::VectorXd q_meas = nominal();
  source.activate(q_meas);
  PoseTarget target = fk(source, nominal());
  target.position += Eigen::Vector3d(0.5, 0.3, -0.2);
  double clamped = 0.0;
  for (int update = 0; update < 500; ++update) {
    source.update(q_meas, kDt, &target);
    for (int index = 0; index < 7; ++index) {
      ASSERT_LE(std::abs(source.q_out()[index] - q_meas[index]), gap[index] + 1e-12)
          << "joint " << index << " update " << update;
    }
    clamped += source.gap_clamped().sum();
  }
  EXPECT_GT(clamped, 0.0);
  // The IK itself ran far past the gap; only the output is held.
  EXPECT_GT((source.q_ik() - q_meas).cwiseAbs().maxCoeff(), 3.0 * gap.maxCoeff());
}

TEST(PositionSourceIk, GapClampHoldsAcrossATargetJumpWhileTheArmTracks) {
  const auto params = ik_params();
  const Eigen::VectorXd gap = gap_of(params);
  PositionSource source = make_source(params);
  Eigen::VectorXd q_meas = nominal();
  source.activate(q_meas);
  PoseTarget near = fk(source, nominal());
  near.position.x() += 0.005;
  PoseTarget far = near;
  far.position += Eigen::Vector3d(-0.4, 0.4, 0.3);
  far.orientation = far.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(1.5, Eigen::Vector3d(1.0, 1.0, 0.0).normalized()));
  for (int update = 0; update < 1500; ++update) {
    const PoseTarget &target = update < 300 ? near : far;
    source.update(q_meas, kDt, &target);
    for (int index = 0; index < 7; ++index) {
      ASSERT_LE(std::abs(source.q_out()[index] - q_meas[index]), gap[index] + 1e-12)
          << "joint " << index << " update " << update;
    }
    q_meas += 0.3 * (source.q_out() - q_meas);
  }
}

TEST(PositionSourceIk, OutputStepIsClampedPerJoint) {
  auto params = ik_params();
  params.max_spring_torque_nm.assign(7, 1e6);  // only the step clamp acts
  params.max_joint_step_rad = {0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007};
  PositionSource source = make_source(params);
  const Eigen::VectorXd q_meas = nominal();
  source.activate(q_meas);
  PoseTarget target = fk(source, nominal());
  target.position += Eigen::Vector3d(-0.2, 0.3, 0.1);
  Eigen::VectorXd previous = source.q_out();
  double clamped = 0.0;
  for (int update = 0; update < 300; ++update) {
    source.update(q_meas, kDt, &target);
    for (int index = 0; index < 7; ++index) {
      ASSERT_LE(std::abs(source.q_out()[index] - previous[index]),
                params.max_joint_step_rad[index] + 1e-15)
          << "joint " << index << " update " << update;
    }
    clamped += source.step_clamped().sum();
    previous = source.q_out();
  }
  EXPECT_GT(clamped, 0.0);
}

// The arm itself moving faster than max_joint_step_rad per update: the step limit gives way, the
// gap holds (A5-A7's URDF velocity limits, 2.27-2.36 rad/s, exceed the default 2 rad/s).
TEST(PositionSourceClamps, GapHoldsWhileTheArmOutrunsTheStepLimit) {
  struct Case {
    const char *name;
    bool targets;  // false: the stale fallback
    int joint;
    double speed_rad_s;
  };
  const Case cases[] = {
      {"stale A7 2.35 rad/s", false, 6, -2.35},
      {"streaming A7 2.35 rad/s", true, 6, 2.35},
  };
  for (const Case &c : cases) {
    const auto params = ik_params();
    const Eigen::VectorXd gap = gap_of(params);
    PositionSource source = make_source(params);
    Eigen::VectorXd q_meas = nominal();
    source.activate(q_meas);
    PoseTarget target = fk(source, nominal());
    target.position.z() += 0.02;
    double gap_clamped = 0.0;
    double step_clamped = 0.0;
    double worst = 0.0;
    for (int update = 0; update < 400; ++update) {
      q_meas[c.joint] += c.speed_rad_s * kDt;
      source.update(q_meas, kDt, c.targets ? &target : nullptr);
      for (int index = 0; index < 7; ++index) {
        ASSERT_LE(std::abs(source.q_out()[index] - q_meas[index]), gap[index] + 1e-12)
            << c.name << ": joint " << index << " update " << update;
      }
      worst = std::max(worst, std::abs(source.q_out()[c.joint] - q_meas[c.joint]));
      gap_clamped += source.gap_clamped()[c.joint];
      step_clamped += source.step_clamped()[c.joint];
    }
    EXPECT_GT(step_clamped, 0.0) << c.name;
    EXPECT_GT(gap_clamped, 0.0) << c.name;
    EXPECT_NEAR(worst, gap[c.joint], 1e-12) << c.name;
  }
}

TEST(PositionSourceIk, JointLimitMarginHolds) {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  urdf_limits(&lower, &upper);
  for (const double margin : {0.05, 0.2}) {
    auto params = ik_params();
    params.joint_limit_margin_rad = margin;
    PositionSource source = make_source(params);
    std::mt19937 random(11);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    double pressed = std::numeric_limits<double>::infinity();
    for (int trial = 0; trial < 10; ++trial) {
      const Eigen::VectorXd q_meas = nominal();
      source.activate(q_meas);
      PoseTarget target;
      target.position = 1.5 * Eigen::Vector3d(unit(random), unit(random), unit(random));
      target.orientation = Eigen::Quaterniond(unit(random), unit(random), unit(random),
                                              unit(random));
      for (int update = 0; update < 300; ++update) {
        source.update(q_meas, kDt, &target);
        const Eigen::VectorXd &q_ik = source.q_ik();
        for (int index = 0; index < 7; ++index) {
          ASSERT_GE(q_ik[index], lower[index] + margin - 1e-12) << "joint " << index;
          ASSERT_LE(q_ik[index], upper[index] - margin + 1e-12) << "joint " << index;
          pressed = std::min(pressed, std::min(q_ik[index] - (lower[index] + margin),
                                               (upper[index] - margin) - q_ik[index]));
        }
      }
    }
    EXPECT_LT(pressed, 1e-9) << "no joint reached its margin, the test proves nothing";
  }
}

TEST(PositionSourceIk, StaleTargetFallsBackAndRecoversLikeAFreshActivation) {
  const auto params = ik_params();
  PositionSource source = make_source(params);
  const Eigen::VectorXd q_meas = nominal();
  source.activate(q_meas);
  EXPECT_TRUE(source.stale());
  PoseTarget target = fk(source, nominal());
  target.position += Eigen::Vector3d(0.05, 0.0, -0.03);
  for (int update = 0; update < 100; ++update) {
    source.update(q_meas, kDt, &target);
    ASSERT_FALSE(source.stale());
  }
  ASSERT_GT((source.q_out() - q_meas).cwiseAbs().maxCoeff(), 0.01);

  int stale_at = -1;
  int timed_out_events = 0;
  Eigen::VectorXd previous = source.q_out();
  for (int update = 1; update <= 200; ++update) {
    source.update(q_meas, kDt, nullptr);
    if (source.events() & PositionSource::kTargetTimedOut) ++timed_out_events;
    if (stale_at < 0 && source.stale()) stale_at = update;
    for (int index = 0; index < 7; ++index) {
      ASSERT_LE(std::abs(source.q_out()[index] - previous[index]), 0.004 + 1e-15);
    }
    previous = source.q_out();
  }
  EXPECT_NEAR(stale_at, 50, 1);  // 0.1 s of 2 ms updates, summed in floating point
  EXPECT_EQ(timed_out_events, 1);
  EXPECT_TRUE(source.stale());
  EXPECT_TRUE(bit_identical(source.q_out(), q_meas));
  EXPECT_TRUE(bit_identical(source.q_ik(), q_meas));

  PositionSource fresh = make_source(params);
  fresh.activate(q_meas);
  PoseTarget next = target;
  next.position.y() += 0.02;
  for (int update = 0; update < 20; ++update) {
    source.update(q_meas, kDt, &next);
    fresh.update(q_meas, kDt, &next);
    ASSERT_FALSE(source.stale());
    ASSERT_TRUE(bit_identical(source.q_ik(), fresh.q_ik())) << "update " << update;
    ASSERT_TRUE(bit_identical(source.q_out(), fresh.q_out())) << "update " << update;
  }
}

TEST(PositionSourceIk, NonFiniteInputsFallBack) {
  PositionSource source = make_source(ik_params());
  const Eigen::VectorXd q_meas = nominal();
  source.activate(q_meas);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  PoseTarget bad;
  bad.position = Eigen::Vector3d(0.5, nan, 0.5);
  source.update(q_meas, kDt, &bad);
  EXPECT_TRUE(source.events() & PositionSource::kTargetNonFinite);
  EXPECT_TRUE(source.stale());
  EXPECT_TRUE(bit_identical(source.q_out(), q_meas));

  PoseTarget good = fk(source, nominal());
  good.position.z() += 0.05;
  for (int update = 0; update < 20; ++update) source.update(q_meas, kDt, &good);
  const Eigen::VectorXd q_ik_before = source.q_ik();
  PositionSource reference = make_source(ik_params());
  reference.activate(q_meas);
  for (int update = 0; update < 20; ++update) reference.update(q_meas, kDt, &good);
  for (const auto &orientation :
       {Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0), Eigen::Quaterniond(inf, 0.0, 0.0, 1.0),
        Eigen::Quaterniond(nan, 0.0, 0.0, 0.0)}) {
    PoseTarget broken = good;
    broken.orientation = orientation;
    source.update(q_meas, kDt, &broken);
    EXPECT_TRUE(source.events() & PositionSource::kTargetNonFinite);
    EXPECT_FALSE(source.stale());
    reference.update(q_meas, kDt, nullptr);
    EXPECT_TRUE(bit_identical(source.q_ik(), reference.q_ik()));
  }
  EXPECT_FALSE(bit_identical(source.q_ik(), q_ik_before));

  const Eigen::VectorXd out_before = source.q_out();
  for (const double dt : {nan, inf, 0.0, -kDt}) {
    source.update(q_meas, dt, &good);
    EXPECT_TRUE(source.events() & PositionSource::kInvalidPeriod);
    EXPECT_TRUE(source.stale());
    EXPECT_TRUE(source.q_out().allFinite());
  }
  EXPECT_LE((source.q_out() - out_before).cwiseAbs().maxCoeff(), 4 * 0.004 + 1e-15);
  source.update(q_meas, kDt, &good);
  EXPECT_FALSE(source.stale());
  EXPECT_EQ(source.events(), 0u);
}

TEST(PositionSourceIk, ActivationResetsEverything) {
  PositionSource source = make_source(ik_params());
  source.activate(nominal());
  PoseTarget target = fk(source, nominal());
  target.position.x() += 0.1;
  for (int update = 0; update < 100; ++update) source.update(nominal(), kDt, &target);
  ASSERT_FALSE(source.stale());

  Eigen::VectorXd q2 = nominal();
  q2[3] -= 0.3;
  q2[5] += 0.2;
  source.activate(q2);
  EXPECT_TRUE(bit_identical(source.q_out(), q2));
  EXPECT_TRUE(bit_identical(source.q_ik(), q2));
  EXPECT_TRUE(source.stale());
  source.update(q2, kDt, nullptr);
  EXPECT_TRUE(bit_identical(source.q_out(), q2));
  EXPECT_TRUE(source.stale());

  PositionSource fresh = make_source(ik_params());
  fresh.activate(q2);
  fresh.update(q2, kDt, nullptr);
  for (int update = 0; update < 50; ++update) {
    source.update(q2, kDt, &target);
    fresh.update(q2, kDt, &target);
    ASSERT_TRUE(bit_identical(source.q_out(), fresh.q_out())) << "update " << update;
  }
}

TEST(PositionSourceIk, MissingFrameOrModelFallsBackToMeasured) {
  auto params = ik_params();
  params.end_effector_frame = "lbr_no_such_frame";
  PositionSource source;
  std::string error;
  ASSERT_TRUE(source.load_model(urdf(), kJoints, &error)) << error;
  ASSERT_TRUE(source.configure(7, params, &error)) << error;
  EXPECT_FALSE(source.kinematics_ready());
  EXPECT_NE(source.kinematics_error().find("lbr_no_such_frame"), std::string::npos);
  PoseTarget target;
  target.position = Eigen::Vector3d(0.5, 0.0, 0.5);
  source.activate(nominal());
  Eigen::VectorXd q = nominal();
  for (int update = 0; update < 10; ++update) {
    q[0] += 0.001;
    source.update(q, kDt, &target);
    EXPECT_TRUE(source.events() & PositionSource::kNoKinematics);
    EXPECT_TRUE(source.stale());
    EXPECT_TRUE(bit_identical(source.q_out(), q));
  }

  PositionSource no_model;
  ASSERT_TRUE(no_model.configure(7, ik_params(), &error)) << error;
  EXPECT_FALSE(no_model.kinematics_ready());
  no_model.activate(nominal());
  no_model.update(nominal(), kDt, &target);
  EXPECT_TRUE(no_model.events() & PositionSource::kNoKinematics);
  EXPECT_TRUE(bit_identical(no_model.q_out(), nominal()));
}

// crisp reads the target in the model's world frame; a base_frame offset from it would put the
// IK on another pose, so the output falls back to the measured position instead.
TEST(PositionSourceIk, BaseFrameOffsetFromTheWorldFallsBack) {
  std::string mounted = urdf();
  const std::string child = "<child link=\"lbr_link_0\"/>";
  const auto at = mounted.find(child);
  ASSERT_NE(at, std::string::npos);
  mounted.insert(at + child.size(), "<origin xyz=\"0.1 0 0.2\" rpy=\"0 0 0.3\"/>");
  PositionSource source;
  std::string error;
  ASSERT_TRUE(source.load_model(mounted, kJoints, &error)) << error;
  ASSERT_TRUE(source.configure(7, ik_params(), &error)) << error;
  EXPECT_FALSE(source.kinematics_ready());
  EXPECT_NE(source.kinematics_error().find("not the model's world"), std::string::npos)
      << source.kinematics_error();
  source.activate(nominal());
  PoseTarget target;
  target.position = Eigen::Vector3d(0.5, 0.0, 0.5);
  source.update(nominal(), kDt, &target);
  EXPECT_TRUE(source.events() & PositionSource::kNoKinematics);
  EXPECT_TRUE(bit_identical(source.q_out(), nominal()));
}

// The check below is only worth something if an allocation would actually trip it.
TEST(PositionSourceRealtime, NoMallocCheckIsArmed) {
  EXPECT_DEATH(
      {
        Eigen::internal::set_is_malloc_allowed(false);
        Eigen::VectorXd allocated(16);
        allocated.setZero();
      },
      "heap allocation is forbidden");
}

TEST(PositionSourceRealtime, UpdateDoesNotAllocate) {
  PositionSource source = make_source(ik_params());
  const Eigen::VectorXd q_meas = nominal();
  PoseTarget target = fk(source, nominal());
  target.position.x() += 0.2;
  PoseTarget bad = target;
  bad.position.x() = std::numeric_limits<double>::quiet_NaN();
  Eigen::internal::set_is_malloc_allowed(false);
  source.activate(q_meas);
  for (int update = 0; update < 200; ++update) {
    // Every branch: targets, a pause that goes stale, a bad target, a bad period.
    const PoseTarget *next = update < 50 ? &target : update == 120 ? &bad
                             : update > 150 ? &target : nullptr;
    source.update(q_meas, update == 160 ? 0.0 : kDt, next);
  }
  Eigen::internal::set_is_malloc_allowed(true);
  EXPECT_TRUE(source.q_out().allFinite());
}

}  // namespace
}  // namespace pixi_kuka_ros2
