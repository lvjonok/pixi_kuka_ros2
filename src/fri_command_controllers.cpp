#include <cmath>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace pixi_kuka_ros2 {

class FRIPositionPassthroughController : public controller_interface::ControllerInterface {
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
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override {
    return mirror_measured_positions() ? controller_interface::CallbackReturn::SUCCESS
                                       : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time &, const rclcpp::Duration &) override {
    return mirror_measured_positions() ? controller_interface::return_type::OK
                                       : controller_interface::return_type::ERROR;
  }

 private:
  bool mirror_measured_positions() {
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
      if (!command_interfaces_[index].set_value(measured_position.value())) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Failed to write the position command for joint '%s'.", joints_[index].c_str());
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> joints_;
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
