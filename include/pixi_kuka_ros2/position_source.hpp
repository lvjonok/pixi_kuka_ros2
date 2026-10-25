#pragma once

// What FRIPositionPassthroughController writes into the FRI position field under the torque
// overlay (docs/position_source.md). ROS-free so it can be unit-tested: the controller owns the
// interfaces, the target subscription and the diagnostics; this owns the law.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// pinocchio/fwd.hpp first: it sets the Boost.MPL limits its variant needs.
#include <pinocchio/fwd.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace pixi_kuka_ros2 {

// The values are the `mode` slot of ~/state.
enum class PositionSourceMode : int { kMeasured = 0, kLagged = 1, kIkTarget = 2 };

bool parse_position_source_mode(const std::string &text, PositionSourceMode *mode);
const char *to_string(PositionSourceMode mode);

struct PositionSourceParams {
  PositionSourceMode mode = PositionSourceMode::kMeasured;
  double lag_time_constant_s = 0.008;
  std::string end_effector_frame = "lbr_link_ee";
  std::string base_frame = "lbr_link_0";
  double target_filter = 0.9;
  double target_timeout_s = 0.1;
  double ik_damping = 0.01;
  double nullspace_gain = 0.05;
  double joint_limit_margin_rad = 0.05;
  // One entry per joint. Unused, and not size-checked, in `measured`.
  std::vector<double> max_joint_step_rad = std::vector<double>(7, 0.004);
  std::vector<double> sunrise_joint_stiffness = std::vector<double>(7, 200.0);
  std::vector<double> max_spring_torque_nm = {10.0, 10.0, 10.0, 8.0, 5.0, 5.0, 3.0};
};

// A target pose of end_effector_frame in base_frame. The orientation need not be normalised.
struct PoseTarget {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class PositionSource {
 public:
  // Bits of events(): what the last update() fell back on, for the controller's warnings.
  static constexpr std::uint32_t kTargetNonFinite = 1u << 0;  // a new target was rejected
  static constexpr std::uint32_t kTargetTimedOut = 1u << 1;   // the target just went stale
  static constexpr std::uint32_t kIkNonFinite = 1u << 2;      // IK produced NaN/Inf; restarted
  static constexpr std::uint32_t kNoKinematics = 1u << 3;     // no model or a missing frame
  static constexpr std::uint32_t kInvalidPeriod = 1u << 4;    // dt not finite and positive

  // Not realtime. Builds the model `ik_target` integrates on from a URDF, every joint not in
  // `joints` locked at its neutral value. Each of `joints` must be a 1-DoF joint of the URDF.
  // On failure the source still runs `measured` and `lagged`; `ik_target` falls back.
  bool load_model(const std::string &urdf_xml, const std::vector<std::string> &joints,
                  std::string *error);

  // Not realtime. Validates `params` for `joint_count` joints and preallocates; false with
  // *error if a parameter is invalid. A missing frame, a base_frame that is not the model's world
  // (crisp reads the target in the world frame) and, outside `ik_target`, a joint margin that
  // leaves no range are not errors here: kinematics_ready() turns false and `ik_target` falls
  // back to the measured position (kinematics_error() says why).
  bool configure(std::size_t joint_count, const PositionSourceParams &params, std::string *error);

  // Realtime. q_out = q_ik = q_posture = q_meas, the filtered target cleared: until a target
  // arrives the output is the measured position.
  void activate(const Eigen::VectorXd &q_meas);

  // Realtime, no allocation. q_meas finite (the controller checks); dt in seconds;
  // new_target is nullptr unless a target arrived since the previous update.
  void update(const Eigen::VectorXd &q_meas, double dt, const PoseTarget *new_target);

  const PositionSourceParams &params() const { return params_; }
  bool model_loaded() const { return model_loaded_; }
  bool kinematics_ready() const { return kinematics_ready_; }
  const std::string &kinematics_error() const { return kinematics_error_; }

  // After update(): the output, the mode's unclamped proposal (`ik_target`: q_ik; `lagged`:
  // the low-pass before the clamps; otherwise q_meas), and 0/1 per joint where a clamp acted.
  const Eigen::VectorXd &q_out() const { return q_out_; }
  const Eigen::VectorXd &q_ik() const { return q_ik_; }
  const Eigen::VectorXd &gap_clamped() const { return gap_clamped_; }
  const Eigen::VectorXd &step_clamped() const { return step_clamped_; }
  const Eigen::VectorXd &gap() const { return gap_; }
  // `ik_target` only: the output is the measured fallback (no target yet, stale, or invalid).
  bool stale() const { return stale_; }
  std::uint32_t events() const { return events_; }

  // Forward kinematics of end_effector_frame in base_frame at q (controller joint order).
  // Not realtime; for tests and tools. Requires kinematics_ready().
  bool forward_kinematics(const Eigen::VectorXd &q, PoseTarget *pose);

 private:
  bool ik_step();
  void fall_back_to_measured(const Eigen::VectorXd &q_meas);
  void clamp_and_step(const Eigen::VectorXd &q_meas, const Eigen::VectorXd &proposal);

  PositionSourceParams params_;
  std::size_t joint_count_ = 0;

  // Kinematics: a reduced model with exactly the controller's joints. idx_q_/idx_v_ map the
  // controller's joint order into the model's configuration and tangent vectors.
  bool model_loaded_ = false;
  bool kinematics_ready_ = false;
  std::string kinematics_error_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<int> idx_q_;
  std::vector<int> idx_v_;
  pinocchio::FrameIndex ee_frame_id_ = 0;
  pinocchio::SE3 world_from_base_ = pinocchio::SE3::Identity();
  Eigen::VectorXd q_lower_;  // URDF limits minus the margin, controller order
  Eigen::VectorXd q_upper_;

  // Per-joint limits.
  Eigen::VectorXd max_step_;
  Eigen::VectorXd gap_;

  // State.
  Eigen::VectorXd q_out_;
  Eigen::VectorXd q_ik_;
  Eigen::VectorXd q_posture_;
  Eigen::VectorXd gap_clamped_;
  Eigen::VectorXd step_clamped_;
  bool have_target_ = false;
  bool stale_ = false;
  double since_target_s_ = 0.0;
  std::uint32_t events_ = 0;
  PoseTarget raw_target_;
  Eigen::Vector3d filtered_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond filtered_orientation_ = Eigen::Quaterniond::Identity();

  // IK workspace, preallocated in configure().
  Eigen::VectorXd q_pin_;
  Eigen::VectorXd posture_pull_;  // model tangent order
  Eigen::VectorXd dq_;            // model tangent order
  Eigen::VectorXd proposal_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> rhs_;
  Eigen::Matrix<double, 6, 1> y_;
  Eigen::Matrix<double, 6, 6> jjt_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_;
  Eigen::Matrix3d rotation_error_;
};

}  // namespace pixi_kuka_ros2
