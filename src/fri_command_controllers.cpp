// First: pinocchio's headers must precede anything that pulls in Boost.MPL.
#include "pixi_kuka_ros2/position_source.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace pixi_kuka_ros2 {

// Writes the FRI position field. Under the torque overlay Sunrise's joint spring is centred on
// it, so what it carries is a choice (docs/position_source.md): `measured` (default, the
// behaviour this controller always had), `lagged`, or `ik_target` (the IK of crisp's target).
// The law lives in PositionSource; this class owns the interfaces, the target subscription and
// ~/state.
class FRIPositionPassthroughController : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("joints", {});
      const PositionSourceParams defaults;
      auto_declare<std::string>("position_source", "measured");
      auto_declare<double>("lag_time_constant_s", defaults.lag_time_constant_s);
      auto_declare<std::string>("target_topic", "target_pose");
      auto_declare<std::string>("end_effector_frame", defaults.end_effector_frame);
      auto_declare<std::string>("base_frame", defaults.base_frame);
      auto_declare<double>("target_filter", defaults.target_filter);
      auto_declare<double>("target_timeout_s", defaults.target_timeout_s);
      auto_declare<double>("ik_damping", defaults.ik_damping);
      auto_declare<double>("nullspace_gain", defaults.nullspace_gain);
      auto_declare<double>("joint_limit_margin_rad", defaults.joint_limit_margin_rad);
      auto_declare<std::vector<double>>(
          "sunrise_joint_stiffness", defaults.sunrise_joint_stiffness);
      auto_declare<std::vector<double>>("max_spring_torque_nm", defaults.max_spring_torque_nm);
      // One per joint. A YAML override is declared from the file before this runs, with the
      // file's type and that type fixed, so a dynamically typed declaration here would never
      // take effect; read_parameters still takes a scalar override for every joint.
      auto_declare<std::vector<double>>("max_joint_step_rad", defaults.max_joint_step_rad);
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_node()->get_logger(), "Initialization failed: %s", exception.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : joints_) {
      configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return configuration;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : joints_) {
      configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return configuration;
  }

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    if (joints_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty.");
      return controller_interface::CallbackReturn::ERROR;
    }

    // The model `ik_target` integrates on: the controller manager's robot_description, the same
    // document crisp builds its model from. Without one the other modes still work.
    std::string error;
    source_ = PositionSource();
    if (get_robot_description().empty()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "No robot_description from the controller manager: ik_target will fall back "
                  "to the measured position.");
    } else if (!source_.load_model(get_robot_description(), joints_, &error)) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "No kinematic model (%s): ik_target will fall back to the measured position.",
                  error.c_str());
    }
    if (!configure_source(&error)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    q_meas_.setZero(static_cast<Eigen::Index>(joints_.size()));
    state_publisher_ = std::make_unique<StatePublisher>(
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/state", rclcpp::SystemDefaultsQoS()));
    state_publisher_->lock();
    auto &message = state_publisher_->msg_;
    const std::size_t n = joints_.size();
    const std::string count = std::to_string(n);
    message.layout.dim.resize(1);
    message.layout.dim[0].label = "mode,stale,q_meas[" + count + "],q_ik[" + count +
                                  "],q_out[" + count + "],gap_clamped[" + count +
                                  "],step_clamped[" + count + "]";
    message.layout.dim[0].size = static_cast<std::uint32_t>(2 + 5 * n);
    message.layout.dim[0].stride = static_cast<std::uint32_t>(2 + 5 * n);
    message.layout.data_offset = 0;
    message.data.assign(2 + 5 * n, 0.0);
    state_publisher_->unlock();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override {
    // Parameters are re-read here, so a change takes effect at the next REST -> ARMED. A bad
    // one is FAILURE, not ERROR: the controller stays inactive and configured, and arms again
    // once the parameter is fixed.
    std::string error;
    if (!configure_source(&error)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!read_measured_positions()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    // A target received before this activation is never used.
    last_target_ = received_targets_.load();
    source_.activate(q_meas_);

    const auto &params = source_.params();
    if (params.mode == PositionSourceMode::kIkTarget) {
      RCLCPP_INFO(
          get_node()->get_logger(),
          "position_source: ik_target (%s in %s from '%s', filter %.3f, timeout %.3f s, "
          "damping %.4f, nullspace %.3f, margin %.3f rad).",
          params.end_effector_frame.c_str(), params.base_frame.c_str(),
          target_subscription_->get_topic_name(), params.target_filter, params.target_timeout_s,
          params.ik_damping, params.nullspace_gain, params.joint_limit_margin_rad);
      if (!source_.kinematics_ready()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "ik_target without kinematics (%s): the output stays the measured position.",
                     source_.kinematics_error().c_str());
      }
    } else if (params.mode == PositionSourceMode::kLagged) {
      RCLCPP_INFO(get_node()->get_logger(), "position_source: lagged (tau %.4f s).",
                  params.lag_time_constant_s);
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "position_source: measured.");
    }
    return write_commands() ? controller_interface::CallbackReturn::SUCCESS
                            : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override {
    // ARMED -> REST: joint_trajectory_controller activates in the same switch, after this, and
    // holds the position command it finds there (set_last_command_interface_value_as_state_on_
    // activation, default true). Leave it the measured position, as `measured` always did: q_out
    // can be up to the gap away, and REST would hold that spring torque for as long as it lasts.
    // The interfaces are still claimed here; the controller manager releases them afterwards.
    if (!read_measured_positions()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Deactivating without a measured position: the position command keeps the "
                   "last output.");
      return controller_interface::CallbackReturn::SUCCESS;
    }
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!command_interfaces_[index].set_value(q_meas_[static_cast<Eigen::Index>(index)])) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to leave the measured position in the command of joint '%s'.",
                     joints_[index].c_str());
      }
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time &, const rclcpp::Duration &period) override {
    if (!read_measured_positions()) {
      return controller_interface::return_type::ERROR;
    }

    const TargetSample &sample = *target_buffer_.readFromRT();
    const bool new_target = sample.seq > last_target_;
    if (new_target) {
      last_target_ = sample.seq;
      target_.position = Eigen::Vector3d(sample.position[0], sample.position[1],
                                         sample.position[2]);
      target_.orientation.coeffs() << sample.orientation[0], sample.orientation[1],
          sample.orientation[2], sample.orientation[3];
    }
    source_.update(q_meas_, period.seconds(), new_target ? &target_ : nullptr);
    warn_about(source_.events());

    const bool written = write_commands();
    publish_state();
    return written ? controller_interface::return_type::OK
                   : controller_interface::return_type::ERROR;
  }

 private:
  // What the subscription hands the realtime loop: plain data, so a read never allocates.
  // seq counts received targets; 0 is "none".
  struct TargetSample {
    std::uint64_t seq = 0;
    double position[3] = {0.0, 0.0, 0.0};
    double orientation[4] = {0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  };
  using StatePublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;

  bool read_parameters(PositionSourceParams *params, std::string *topic, std::string *error) {
    const auto node = get_node();
    const std::string mode = node->get_parameter("position_source").as_string();
    if (!parse_position_source_mode(mode, &params->mode)) {
      *error = "position_source '" + mode + "' is not one of measured, lagged, ik_target.";
      return false;
    }
    params->lag_time_constant_s = node->get_parameter("lag_time_constant_s").as_double();
    *topic = node->get_parameter("target_topic").as_string();
    params->end_effector_frame = node->get_parameter("end_effector_frame").as_string();
    params->base_frame = node->get_parameter("base_frame").as_string();
    params->target_filter = node->get_parameter("target_filter").as_double();
    params->target_timeout_s = node->get_parameter("target_timeout_s").as_double();
    params->ik_damping = node->get_parameter("ik_damping").as_double();
    params->nullspace_gain = node->get_parameter("nullspace_gain").as_double();
    params->joint_limit_margin_rad = node->get_parameter("joint_limit_margin_rad").as_double();
    params->sunrise_joint_stiffness =
        node->get_parameter("sunrise_joint_stiffness").as_double_array();
    params->max_spring_torque_nm = node->get_parameter("max_spring_torque_nm").as_double_array();

    const auto step = node->get_parameter("max_joint_step_rad");
    switch (step.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        params->max_joint_step_rad.assign(joints_.size(), step.as_double());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        params->max_joint_step_rad.assign(joints_.size(), static_cast<double>(step.as_int()));
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        params->max_joint_step_rad = step.as_double_array();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        params->max_joint_step_rad.clear();
        for (const auto value : step.as_integer_array()) {
          params->max_joint_step_rad.push_back(static_cast<double>(value));
        }
        break;
      default:
        *error = "max_joint_step_rad must be a double or a double array.";
        return false;
    }
    if (topic->empty()) {
      *error = "target_topic must not be empty.";
      return false;
    }
    return true;
  }

  // Reads every parameter into the source and (re)subscribes when the topic changed. That is the
  // only subscription work at activation; the callback reads base_frame for itself.
  bool configure_source(std::string *error) {
    PositionSourceParams params;
    std::string topic;
    try {
      if (!read_parameters(&params, &topic, error)) {
        return false;
      }
    } catch (const std::exception &exception) {
      *error = std::string("Reading the position_source parameters failed: ") + exception.what();
      return false;
    }
    if (!source_.configure(joints_.size(), params, error)) {
      return false;
    }
    if (!target_subscription_ || topic != subscribed_topic_) {
      target_subscription_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, rclcpp::QoS(1),
          [this, topic](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
            // crisp's cartesian_impedance_controller ignores the topic while it has more than one
            // publisher and holds its last target. So does this: no new target, and the output
            // goes stale (the measured position) instead of chasing a pose crisp is not.
            const std::size_t publishers = get_node()->count_publishers(topic);
            if (publishers > 1) {
              RCLCPP_WARN_THROTTLE(
                  get_node()->get_logger(), *get_node()->get_clock(), 2000,
                  "Ignoring targets: '%s' has %zu publishers (crisp ignores it too).",
                  topic.c_str(), publishers);
              return;
            }
            // crisp reads the pose without looking at the frame; so does this, but says so.
            if (!message->header.frame_id.empty()) {
              const std::string base_frame = get_node()->get_parameter("base_frame").as_string();
              if (message->header.frame_id != base_frame) {
                RCLCPP_WARN_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 5000,
                    "Target in frame '%s', read as base_frame '%s' (as crisp does).",
                    message->header.frame_id.c_str(), base_frame.c_str());
              }
            }
            TargetSample sample;
            sample.position[0] = message->pose.position.x;
            sample.position[1] = message->pose.position.y;
            sample.position[2] = message->pose.position.z;
            sample.orientation[0] = message->pose.orientation.x;
            sample.orientation[1] = message->pose.orientation.y;
            sample.orientation[2] = message->pose.orientation.z;
            sample.orientation[3] = message->pose.orientation.w;
            sample.seq = received_targets_.fetch_add(1) + 1;
            target_buffer_.writeFromNonRT(sample);
          });
      subscribed_topic_ = topic;
    }
    return true;
  }

  bool read_measured_positions() {
    if (command_interfaces_.size() != joints_.size() || state_interfaces_.size() != joints_.size()) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Expected %zu position command and state interfaces, got %zu and %zu.",
          joints_.size(), command_interfaces_.size(), state_interfaces_.size());
      return false;
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const auto measured_position = state_interfaces_[index].get_optional();
      if (!measured_position.has_value() || !std::isfinite(measured_position.value())) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "No finite position state for joint '%s'.", joints_[index].c_str());
        return false;
      }
      q_meas_[static_cast<Eigen::Index>(index)] = measured_position.value();
    }
    return true;
  }

  bool write_commands() {
    const auto &q_out = source_.q_out();
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!command_interfaces_[index].set_value(q_out[static_cast<Eigen::Index>(index)])) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Failed to write the position command for joint '%s'.", joints_[index].c_str());
        return false;
      }
    }
    return true;
  }

  void warn_about(std::uint32_t events) {
    if (events == 0) {
      return;
    }
    const auto logger = get_node()->get_logger();
    auto &clock = *get_node()->get_clock();
    if (events & PositionSource::kTargetTimedOut) {
      RCLCPP_WARN_THROTTLE(
          logger, clock, 1000,
          "No target for %.3f s: the position field returns to the measured position until "
          "one arrives.", source_.params().target_timeout_s);
    }
    if (events & PositionSource::kTargetNonFinite) {
      RCLCPP_WARN_THROTTLE(logger, clock, 1000, "Rejected a non-finite target pose.");
    }
    if (events & PositionSource::kIkNonFinite) {
      RCLCPP_WARN_THROTTLE(
          logger, clock, 1000,
          "IK went non-finite: the measured position until the next target.");
    }
    if (events & PositionSource::kNoKinematics) {
      RCLCPP_WARN_THROTTLE(
          logger, clock, 5000, "ik_target without kinematics (%s): the measured position.",
          source_.kinematics_error().c_str());
    }
    if (events & PositionSource::kInvalidPeriod) {
      RCLCPP_WARN_THROTTLE(
          logger, clock, 1000, "Non-positive or non-finite period: the measured position.");
    }
  }

  // [mode, stale, q_meas[n], q_ik[n], q_out[n], gap_clamped[n], step_clamped[n]].
  void publish_state() {
    if (!state_publisher_ || !state_publisher_->trylock()) {
      return;
    }
    auto &data = state_publisher_->msg_.data;
    const auto n = static_cast<Eigen::Index>(joints_.size());
    data[0] = static_cast<double>(static_cast<int>(source_.params().mode));
    data[1] = source_.stale() ? 1.0 : 0.0;
    const Eigen::VectorXd *const blocks[] = {&q_meas_, &source_.q_ik(), &source_.q_out(),
                                             &source_.gap_clamped(), &source_.step_clamped()};
    double *slot = data.data() + 2;
    for (const Eigen::VectorXd *values : blocks) {
      Eigen::Map<Eigen::VectorXd>(slot, n) = *values;
      slot += n;
    }
    state_publisher_->unlockAndPublish();
  }

  std::vector<std::string> joints_;
  PositionSource source_;
  Eigen::VectorXd q_meas_;
  PoseTarget target_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
  std::string subscribed_topic_;
  realtime_tools::RealtimeBuffer<TargetSample> target_buffer_;
  std::atomic<std::uint64_t> received_targets_{0};
  std::uint64_t last_target_ = 0;

  std::unique_ptr<StatePublisher> state_publisher_;
};

class ZeroEffortController : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("joints", {});
    } catch (const std::exception &exception) {
      RCLCPP_ERROR(get_node()->get_logger(), "Initialization failed: %s", exception.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : joints_) {
      configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return configuration;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    if (joints_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty.");
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override {
    return write_zero_effort() ? controller_interface::CallbackReturn::SUCCESS
                               : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time &, const rclcpp::Duration &) override {
    return write_zero_effort() ? controller_interface::return_type::OK
                               : controller_interface::return_type::ERROR;
  }

 private:
  bool write_zero_effort() {
    if (command_interfaces_.size() != joints_.size()) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Expected %zu effort command interfaces, got %zu.", joints_.size(),
          command_interfaces_.size());
      return false;
    }
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!command_interfaces_[index].set_value(0.0)) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Failed to write zero effort for joint '%s'.", joints_[index].c_str());
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> joints_;
};

}  // namespace pixi_kuka_ros2

PLUGINLIB_EXPORT_CLASS(
    pixi_kuka_ros2::FRIPositionPassthroughController,
    controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(
    pixi_kuka_ros2::ZeroEffortController,
    controller_interface::ControllerInterface)
