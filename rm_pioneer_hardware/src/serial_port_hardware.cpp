// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_pioneer_hardware/serial_port_hardware.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rm_pioneer_hardware
{
CallbackReturn SerialPortHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("SerialPortHardware"), "Initializing...");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  serial_driver_ = std::make_unique<RMSerialDriver>(info.hardware_parameters);

  filter_.setDoBiasEstimation(true);
  filter_.setDoAdaptiveGain(true);
  filter_.setGainAcc(0.01);
  filter_.setBiasAlpha(0.01);

  hw_joint_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_joint_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPortHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPortHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPortHardware"), "Joint '%s' has %zu state interface. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SerialPortHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    // Initialize joint coefficients
    hw_joint_coefficients_.emplace_back(
      std::stod(joint.parameters.at("effort_actuator_coefficient")));
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SerialPortHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_joint_states_.size(); i++) {
    hw_joint_states_[i] = 0;
    hw_joint_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialPortHardware"), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SerialPortHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_states_[i]));
  }

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SerialPortHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn SerialPortHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (uint i = 0; i < hw_joint_states_.size(); i++) {
    hw_joint_states_[i] = hw_joint_states_[i];
  }

  // set default value for sensor
  if (std::isnan(hw_sensor_states_[0])) {
    hw_sensor_states_[0] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SerialPortHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}
CallbackReturn SerialPortHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SerialPortHardware::read()
{
  // Read imu raw data
  serial_driver_->sendRequest();
  auto packet = serial_driver_->readData();
  auto time = std::chrono::steady_clock::now();

  // Initialize
  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return hardware_interface::return_type::OK;
  }

  // Update the filter
  double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(time - time_prev_).count() / 1e9;
  time_prev_ = time;
  filter_.update(
    packet.linear_acceleration_x, packet.linear_acceleration_y, packet.linear_acceleration_z,
    packet.angular_velocity_x, packet.angular_velocity_y, packet.angular_velocity_z, dt);

  // Get the orientation:
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  auto q = tf2::Quaternion(q1, q2, q3, q0);

  // Update the state
  // Orientation
  hw_sensor_states_[0] = q.x();
  hw_sensor_states_[1] = q.y();
  hw_sensor_states_[2] = q.z();
  hw_sensor_states_[3] = q.w();
  // Linear acceleration
  hw_sensor_states_[4] = packet.linear_acceleration_x;
  hw_sensor_states_[5] = packet.linear_acceleration_y;
  hw_sensor_states_[6] = packet.linear_acceleration_z;
  // Angular velocity
  hw_sensor_states_[7] = packet.angular_velocity_x - filter_.getAngularVelocityBiasX();
  hw_sensor_states_[8] = packet.angular_velocity_y - filter_.getAngularVelocityBiasY();
  hw_sensor_states_[9] = packet.angular_velocity_z - filter_.getAngularVelocityBiasZ();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SerialPortHardware::write()
{
  double pitch_command = hw_joint_coefficients_[0] * hw_joint_commands_[0];
  double yaw_command = hw_joint_coefficients_[1] * hw_joint_commands_[1];
  bool shoot_command = hw_joint_commands_[2];
  serial_driver_->writeCommand(pitch_command, yaw_command, shoot_command);

  return hardware_interface::return_type::OK;
}

}  // namespace rm_pioneer_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rm_pioneer_hardware::SerialPortHardware, hardware_interface::SystemInterface)
