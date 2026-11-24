#pragma once

// The law of what FRIPositionPassthroughController writes into the FRI position field: the
// differential IK of a Cartesian target, held near the measured position
// (docs/position_source.md).

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

struct PositionSourceParams {
  std::string end_effector_frame = "lbr_link_ee";
  std::string base_frame = "lbr_link_0";
  double target_filter = 0.9;
  double target_timeout_s = 0.1;
  double ik_damping = 0.01;
  double nullspace_gain = 0.05;
  double joint_limit_margin_rad = 0.05;
  // One entry per joint.
  std::vector<double> max_joint_step_rad = std::vector<double>(7, 0.004);
  std::vector<double> sunrise_joint_stiffness = {1000.0, 1000.0, 1000.0, 500.0,
                                                 500.0,  500.0,  500.0};
  std::vector<double> max_spring_torque_nm = {5.0, 5.0, 5.0, 4.0, 2.5, 2.5, 1.5};
};

// A target pose of end_effector_frame in base_frame. The orientation need not be normalised.
struct PoseTarget {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class PositionSource {
 public:
  // Bits of events(): what the last update() fell back on.
  static constexpr std::uint32_t kTargetNonFinite = 1u << 0;
  static constexpr std::uint32_t kTargetTimedOut = 1u << 1;
  static constexpr std::uint32_t kIkNonFinite = 1u << 2;
  static constexpr std::uint32_t kNoKinematics = 1u << 3;
  static constexpr std::uint32_t kInvalidPeriod = 1u << 4;

  // Not realtime. The model the IK integrates on: the URDF with every joint not in `joints`
  // locked at neutral. On failure the output is the measured position.
  bool load_model(const std::string &urdf_xml, const std::vector<std::string> &joints,
                  std::string *error);

  // Not realtime. False with *error if a parameter is invalid. A missing model or frame and a
  // base_frame that is not the model's world are not errors: kinematics_ready() turns false and
  // the output is the measured position (kinematics_error() says why).
  bool configure(std::size_t joint_count, const PositionSourceParams &params, std::string *error);

  // Realtime. Until a target arrives the output is the measured position.
  void activate(const Eigen::VectorXd &q_meas);

  // Realtime, no allocation. q_meas must be finite; dt in seconds; new_target is nullptr unless
  // a target arrived since the previous update.
  void update(const Eigen::VectorXd &q_meas, double dt, const PoseTarget *new_target);

  const PositionSourceParams &params() const { return params_; }
  bool model_loaded() const { return model_loaded_; }
  bool kinematics_ready() const { return kinematics_ready_; }
  const std::string &kinematics_error() const { return kinematics_error_; }

  // After update(): the output, the IK's configuration before the clamps (the measured position
  // while stale()), and 0/1 per joint where a clamp acted.
  const Eigen::VectorXd &q_out() const { return q_out_; }
  const Eigen::VectorXd &q_ik() const { return q_ik_; }
  const Eigen::VectorXd &gap_clamped() const { return gap_clamped_; }
  const Eigen::VectorXd &step_clamped() const { return step_clamped_; }
  const Eigen::VectorXd &gap() const { return gap_; }
  // The output is the measured position: no target yet, a stale one, or an invalid input.
  bool stale() const { return stale_; }
  std::uint32_t events() const { return events_; }

  // Not realtime. end_effector_frame in base_frame at q; false unless kinematics_ready().
  bool forward_kinematics(const Eigen::VectorXd &q, PoseTarget *pose);

 private:
  bool ik_step();
  void fall_back_to_measured(const Eigen::VectorXd &q_meas);
  void clamp_and_step(const Eigen::VectorXd &q_meas, const Eigen::VectorXd &proposal);

  PositionSourceParams params_;
  std::size_t joint_count_ = 0;

  // idx_q_/idx_v_: the controller's joint order to the model's configuration and tangent
  // indices.
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

  Eigen::VectorXd max_step_;
  Eigen::VectorXd gap_;

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

  Eigen::VectorXd q_pin_;
  Eigen::VectorXd posture_pull_;  // model tangent order
  Eigen::VectorXd dq_;            // model tangent order
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> rhs_;
  Eigen::Matrix<double, 6, 1> y_;
  Eigen::Matrix<double, 6, 6> jjt_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_;
  Eigen::Matrix3d rotation_error_;
};

}  // namespace pixi_kuka_ros2
