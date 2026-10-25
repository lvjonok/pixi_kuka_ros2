#include "pixi_kuka_ros2/position_source.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace pixi_kuka_ros2 {

namespace {

bool finite_and_at_least(double value, double minimum) {
  return std::isfinite(value) && value >= minimum;
}

// Copies a per-joint parameter; false (with *error) on a wrong size or a value below `minimum`
// (strictly above it when `strict`).
bool per_joint(const std::vector<double> &values, std::size_t joint_count, const char *name,
               double minimum, bool strict, Eigen::VectorXd *out, std::string *error) {
  if (values.size() != joint_count) {
    *error = std::string(name) + " has " + std::to_string(values.size()) + " values, expected " +
             std::to_string(joint_count) + " (one per joint).";
    return false;
  }
  out->resize(static_cast<Eigen::Index>(joint_count));
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double value = values[index];
    if (!std::isfinite(value) || value < minimum || (strict && value == minimum)) {
      *error = std::string(name) + "[" + std::to_string(index) + "] = " + std::to_string(value) +
               " must be finite and " + (strict ? "> " : ">= ") + std::to_string(minimum) + ".";
      return false;
    }
    (*out)[static_cast<Eigen::Index>(index)] = value;
  }
  return true;
}

}  // namespace

bool parse_position_source_mode(const std::string &text, PositionSourceMode *mode) {
  if (text == "measured") {
    *mode = PositionSourceMode::kMeasured;
  } else if (text == "lagged") {
    *mode = PositionSourceMode::kLagged;
  } else if (text == "ik_target") {
    *mode = PositionSourceMode::kIkTarget;
  } else {
    return false;
  }
  return true;
}

const char *to_string(PositionSourceMode mode) {
  switch (mode) {
    case PositionSourceMode::kMeasured:
      return "measured";
    case PositionSourceMode::kLagged:
      return "lagged";
    case PositionSourceMode::kIkTarget:
      return "ik_target";
  }
  return "unknown";
}

bool PositionSource::load_model(const std::string &urdf_xml,
                                const std::vector<std::string> &joints, std::string *error) {
  model_loaded_ = false;
  kinematics_ready_ = false;
  pinocchio::Model raw;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, raw);
  } catch (const std::exception &exception) {
    *error = std::string("the URDF does not parse: ") + exception.what();
    return false;
  }

  for (std::size_t index = 0; index < joints.size(); ++index) {
    const auto &joint = joints[index];
    if (std::find(joints.begin(), joints.begin() + static_cast<long>(index), joint) !=
        joints.begin() + static_cast<long>(index)) {
      *error = "joint '" + joint + "' is listed twice.";
      return false;
    }
    if (!raw.existJointName(joint)) {
      *error = "joint '" + joint + "' is not in the URDF.";
      return false;
    }
    const auto &model_joint = raw.joints[raw.getJointId(joint)];
    if (model_joint.nq() != 1 || model_joint.nv() != 1) {
      *error = "joint '" + joint + "' is a " + model_joint.shortname() +
               "; only 1-DoF joints are supported.";
      return false;
    }
  }

  // The rest (a gripper's jaws, say) is locked at neutral, as crisp locks it at zero.
  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex joint_id = 1; joint_id < static_cast<pinocchio::JointIndex>(raw.njoints);
       ++joint_id) {
    if (std::find(joints.begin(), joints.end(), raw.names[joint_id]) == joints.end()) {
      locked.push_back(joint_id);
    }
  }
  try {
    model_ = pinocchio::buildReducedModel(raw, locked, pinocchio::neutral(raw));
  } catch (const std::exception &exception) {
    *error = std::string("reducing the model failed: ") + exception.what();
    return false;
  }
  data_ = pinocchio::Data(model_);

  idx_q_.assign(joints.size(), 0);
  idx_v_.assign(joints.size(), 0);
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const auto &model_joint = model_.joints[model_.getJointId(joints[index])];
    idx_q_[index] = model_joint.idx_q();
    idx_v_[index] = model_joint.idx_v();
  }
  q_pin_ = pinocchio::neutral(model_);
  model_loaded_ = true;
  return true;
}

bool PositionSource::configure(std::size_t joint_count, const PositionSourceParams &params,
                               std::string *error) {
  if (joint_count == 0) {
    *error = "no joints.";
    return false;
  }
  if (!finite_and_at_least(params.lag_time_constant_s, 0.0)) {
    *error = "lag_time_constant_s must be finite and >= 0.";
    return false;
  }
  if (!std::isfinite(params.target_filter) || params.target_filter < 0.0 ||
      params.target_filter >= 1.0) {
    *error = "target_filter must be in [0, 1).";
    return false;
  }
  if (!std::isfinite(params.target_timeout_s) || params.target_timeout_s <= 0.0) {
    *error = "target_timeout_s must be finite and > 0.";
    return false;
  }
  if (!finite_and_at_least(params.ik_damping, 0.0)) {
    *error = "ik_damping must be finite and >= 0.";
    return false;
  }
  if (!finite_and_at_least(params.nullspace_gain, 0.0) || params.nullspace_gain > 1.0) {
    *error = "nullspace_gain must be in [0, 1].";
    return false;
  }
  if (!finite_and_at_least(params.joint_limit_margin_rad, 0.0)) {
    *error = "joint_limit_margin_rad must be finite and >= 0.";
    return false;
  }

  const auto n = static_cast<Eigen::Index>(joint_count);
  if (params.mode == PositionSourceMode::kMeasured) {
    // Unused: measured never clamps, so a config written for another arm still passes through.
    max_step_.setZero(n);
    gap_.setZero(n);
  } else {
    Eigen::VectorXd stiffness;
    Eigen::VectorXd torque;
    if (!per_joint(params.max_joint_step_rad, joint_count, "max_joint_step_rad", 0.0, true,
                   &max_step_, error) ||
        !per_joint(params.sunrise_joint_stiffness, joint_count, "sunrise_joint_stiffness", 0.0,
                   true, &stiffness, error) ||
        !per_joint(params.max_spring_torque_nm, joint_count, "max_spring_torque_nm", 0.0, false,
                   &torque, error)) {
      return false;
    }
    gap_ = torque.cwiseQuotient(stiffness);
  }

  params_ = params;
  joint_count_ = joint_count;
  q_out_.setZero(n);
  q_ik_.setZero(n);
  q_posture_.setZero(n);
  gap_clamped_.setZero(n);
  step_clamped_.setZero(n);
  proposal_.setZero(n);

  kinematics_ready_ = false;
  kinematics_error_.clear();
  if (!model_loaded_) {
    kinematics_error_ = "no robot model (robot_description missing or unusable)";
  } else if (static_cast<std::size_t>(model_.nv) != joint_count ||
             idx_q_.size() != joint_count) {
    kinematics_error_ = "the model has " + std::to_string(model_.nv) + " joints, the controller " +
                        std::to_string(joint_count);
  } else if (!model_.existFrame(params.end_effector_frame)) {
    kinematics_error_ = "end_effector_frame '" + params.end_effector_frame + "' is not in the model";
  } else if (!model_.existFrame(params.base_frame)) {
    kinematics_error_ = "base_frame '" + params.base_frame + "' is not in the model";
  } else if (model_.frames[model_.getFrameId(params.base_frame)].parentJoint != 0) {
    kinematics_error_ = "base_frame '" + params.base_frame + "' moves with a joint";
  } else {
    ee_frame_id_ = model_.getFrameId(params.end_effector_frame);
    // Fixed to the universe, so its placement is its pose in the model's world.
    world_from_base_ = model_.frames[model_.getFrameId(params.base_frame)].placement;
    q_lower_.resize(n);
    q_upper_.resize(n);
    for (Eigen::Index index = 0; index < n && kinematics_error_.empty(); ++index) {
      q_lower_[index] = model_.lowerPositionLimit[idx_q_[index]] + params.joint_limit_margin_rad;
      q_upper_[index] = model_.upperPositionLimit[idx_q_[index]] - params.joint_limit_margin_rad;
      if (!(q_lower_[index] <= q_upper_[index])) {
        kinematics_error_ =
            "joint_limit_margin_rad leaves no range for joint " + std::to_string(index);
      }
    }
    if (!kinematics_error_.empty()) {
      // Only ik_target uses the margin; the other modes still run.
      if (params.mode == PositionSourceMode::kIkTarget) {
        *error = kinematics_error_ + ".";
        return false;
      }
    } else if (!world_from_base_.isIdentity(1e-9)) {
      // crisp has no base_frame: it compares the raw target with the end effector's pose in the
      // model's world. Reading the target in an offset base_frame would chase another pose.
      const Eigen::Vector3d &offset = world_from_base_.translation();
      kinematics_error_ =
          "base_frame '" + params.base_frame + "' is not the model's world (offset [" +
          std::to_string(offset.x()) + ", " + std::to_string(offset.y()) + ", " +
          std::to_string(offset.z()) + "] m, rotated " +
          std::to_string(Eigen::AngleAxisd(world_from_base_.rotation()).angle()) +
          " rad), but crisp reads the target in the world frame";
    } else {
      posture_pull_.setZero(model_.nv);
      dq_.setZero(model_.nv);
      jacobian_.setZero(6, model_.nv);
      kinematics_ready_ = true;
    }
  }
  return true;
}

void PositionSource::activate(const Eigen::VectorXd &q_meas) {
  q_out_ = q_meas;
  q_ik_ = q_meas;
  q_posture_ = q_meas;
  gap_clamped_.setZero();
  step_clamped_.setZero();
  have_target_ = false;
  stale_ = params_.mode == PositionSourceMode::kIkTarget;
  since_target_s_ = 0.0;
  events_ = 0;
  filtered_position_.setZero();
  filtered_orientation_.setIdentity();
}

void PositionSource::update(const Eigen::VectorXd &q_meas, double dt,
                            const PoseTarget *new_target) {
  events_ = 0;
  gap_clamped_.setZero();
  step_clamped_.setZero();

  if (params_.mode == PositionSourceMode::kMeasured) {
    // Today's behaviour, bit for bit: no filter, no clamp.
    q_out_ = q_meas;
    q_ik_ = q_meas;
    stale_ = false;
    return;
  }

  const bool ik = params_.mode == PositionSourceMode::kIkTarget;
  if (!std::isfinite(dt) || dt <= 0.0) {
    events_ |= kInvalidPeriod;
    have_target_ = false;
    fall_back_to_measured(q_meas);
    return;
  }

  if (!ik) {
    // q_out += a (q_meas - q_out), a = 1 - exp(-dt/tau): the exact discretisation of a
    // first-order lag at any dt (Drake's pole 0.8816 per 1 ms is tau 7.94 ms). The low-pass
    // integrates on the output itself.
    const double tau = params_.lag_time_constant_s;
    const double alpha = tau > 0.0 ? -std::expm1(-dt / tau) : 1.0;
    proposal_ = q_out_ + alpha * (q_meas - q_out_);
    q_ik_ = proposal_;
    stale_ = false;
    clamp_and_step(q_meas, proposal_);
    return;
  }

  if (!kinematics_ready_) {
    events_ |= kNoKinematics;
    have_target_ = false;
    fall_back_to_measured(q_meas);
    return;
  }

  if (new_target != nullptr) {
    const double norm = new_target->orientation.norm();
    if (new_target->position.allFinite() && new_target->orientation.coeffs().allFinite() &&
        std::isfinite(norm) && norm > 1e-9) {
      raw_target_.position = new_target->position;
      raw_target_.orientation.coeffs() = new_target->orientation.coeffs() / norm;
      if (!have_target_) {
        // The first target after activation, or after a fallback: the filter starts on it and
        // IK restarts from where the arm is.
        filtered_position_ = raw_target_.position;
        filtered_orientation_ = raw_target_.orientation;
        q_ik_ = q_meas;
        have_target_ = true;
      }
      since_target_s_ = 0.0;
    } else {
      events_ |= kTargetNonFinite;
    }
  }

  if (have_target_) {
    since_target_s_ += dt;
    if (since_target_s_ > params_.target_timeout_s) {
      have_target_ = false;
      events_ |= kTargetTimedOut;
    }
  }
  if (!have_target_) {
    fall_back_to_measured(q_meas);
    return;
  }

  // crisp's filter.target_pose, so both act on the same pose: weight on the previous value.
  const double weight = params_.target_filter;
  filtered_position_ = (1.0 - weight) * raw_target_.position + weight * filtered_position_;
  filtered_orientation_ = raw_target_.orientation.slerp(weight, filtered_orientation_);
  filtered_orientation_.normalize();

  if (!ik_step()) {
    events_ |= kIkNonFinite;
    have_target_ = false;
    fall_back_to_measured(q_meas);
    return;
  }
  stale_ = false;
  clamp_and_step(q_meas, q_ik_);
}

bool PositionSource::ik_step() {
  // Integrated on q_ik, never on the measured position: that would reproduce `measured`.
  for (std::size_t index = 0; index < joint_count_; ++index) {
    q_pin_[idx_q_[index]] = q_ik_[static_cast<Eigen::Index>(index)];
  }
  pinocchio::computeJointJacobians(model_, data_, q_pin_);
  pinocchio::updateFramePlacement(model_, data_, ee_frame_id_);
  jacobian_.setZero();
  pinocchio::getFrameJacobian(model_, data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobian_);
  const pinocchio::SE3 &world_from_ee = data_.oMf[ee_frame_id_];

  // e = [p_target - p(q_ik); log3(R_target R(q_ik)^T)], in world (= base) axes.
  error_.head<3>() = world_from_base_.rotation() * filtered_position_ +
                     world_from_base_.translation() - world_from_ee.translation();
  rotation_error_.noalias() = world_from_base_.rotation() *
                              filtered_orientation_.toRotationMatrix() *
                              world_from_ee.rotation().transpose();
  error_.tail<3>() = pinocchio::log3(rotation_error_);

  // dq = J^+ e + (I - J^+ J) k (q_posture - q_ik), J^+ = J^T (J J^T + lambda^2 I)^-1,
  // folded into one 6x6 solve: dq = J^T (J J^T + lambda^2 I)^-1 (e - k J v) + k v.
  const double gain = params_.nullspace_gain;
  for (std::size_t index = 0; index < joint_count_; ++index) {
    const auto joint = static_cast<Eigen::Index>(index);
    posture_pull_[idx_v_[index]] = q_posture_[joint] - q_ik_[joint];
  }
  rhs_.noalias() = jacobian_ * posture_pull_;
  rhs_ = error_ - gain * rhs_;
  jjt_.noalias() = jacobian_ * jacobian_.transpose();
  jjt_.diagonal().array() += params_.ik_damping * params_.ik_damping;
  ldlt_.compute(jjt_);
  y_ = ldlt_.solve(rhs_);
  dq_.noalias() = jacobian_.transpose() * y_;
  dq_ += gain * posture_pull_;

  for (std::size_t index = 0; index < joint_count_; ++index) {
    const auto joint = static_cast<Eigen::Index>(index);
    const double q = q_ik_[joint] + dq_[idx_v_[index]];
    if (!std::isfinite(q)) {
      return false;
    }
    q_ik_[joint] = std::min(std::max(q, q_lower_[joint]), q_upper_[joint]);
  }
  return true;
}

void PositionSource::fall_back_to_measured(const Eigen::VectorXd &q_meas) {
  stale_ = params_.mode == PositionSourceMode::kIkTarget;
  q_ik_ = q_meas;
  clamp_and_step(q_meas, q_meas);
}

void PositionSource::clamp_and_step(const Eigen::VectorXd &q_meas,
                                    const Eigen::VectorXd &proposal) {
  for (Eigen::Index index = 0; index < static_cast<Eigen::Index>(joint_count_); ++index) {
    // |q_out - q_meas| <= max_spring_torque / sunrise_joint_stiffness: the spring torque bound.
    double value = proposal[index];
    if (value < q_meas[index] - gap_[index]) {
      value = q_meas[index] - gap_[index];
      gap_clamped_[index] = 1.0;
    } else if (value > q_meas[index] + gap_[index]) {
      value = q_meas[index] + gap_[index];
      gap_clamped_[index] = 1.0;
    }
    // Then |q_out - q_out_prev| <= max_joint_step_rad.
    const double step = value - q_out_[index];
    if (step > max_step_[index]) {
      value = q_out_[index] + max_step_[index];
      step_clamped_[index] = 1.0;
    } else if (step < -max_step_[index]) {
      value = q_out_[index] - max_step_[index];
      step_clamped_[index] = 1.0;
    }
    // The gap has the last word. When the arm itself moves faster than the step limit, a
    // step-limited q_out would trail it by more than the gap; the step limit gives way instead.
    if (value < q_meas[index] - gap_[index]) {
      value = q_meas[index] - gap_[index];
      gap_clamped_[index] = 1.0;
    } else if (value > q_meas[index] + gap_[index]) {
      value = q_meas[index] + gap_[index];
      gap_clamped_[index] = 1.0;
    }
    q_out_[index] = value;
  }
}

bool PositionSource::forward_kinematics(const Eigen::VectorXd &q, PoseTarget *pose) {
  if (!kinematics_ready_ || static_cast<std::size_t>(q.size()) != joint_count_) {
    return false;
  }
  for (std::size_t index = 0; index < joint_count_; ++index) {
    q_pin_[idx_q_[index]] = q[static_cast<Eigen::Index>(index)];
  }
  pinocchio::framesForwardKinematics(model_, data_, q_pin_);
  const pinocchio::SE3 base_from_ee = world_from_base_.actInv(data_.oMf[ee_frame_id_]);
  pose->position = base_from_ee.translation();
  pose->orientation = Eigen::Quaterniond(base_from_ee.rotation());
  return true;
}

}  // namespace pixi_kuka_ros2
