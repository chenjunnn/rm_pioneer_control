// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_gimbal_controller/rm_gimbal_controller.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <control_toolbox/pid_ros.hpp>
#include <controller_interface/controller_interface.hpp>
#include <forward_command_controller/forward_command_controller.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

// STL
#include <memory>
#include <string>
#include <vector>

namespace rm_gimbal_controller
{
RMGimbalController::RMGimbalController()
: forward_command_controller::ForwardCommandController(),
  logger_(rclcpp::get_logger("rm_gimbal_controller"))
{
  interface_name_ = hardware_interface::HW_IF_EFFORT;
}

CallbackReturn RMGimbalController::on_init()
{
  auto ret = forward_command_controller::ForwardCommandController::on_init();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    // Explicitly set the joints and interface parameter declared by the forward_command_controller
    get_node()->set_parameter(
      rclcpp::Parameter("joints", std::vector<std::string>{"pitch_joint", "yaw_joint"}));
    get_node()->set_parameter(
      rclcpp::Parameter("interface_name", hardware_interface::HW_IF_EFFORT));

    // Get imu sensor name
    sensor_name_ = auto_declare<std::string>("sensor_name", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  pitch_pid_ = std::make_unique<control_toolbox::PidROS>(get_node(), "pitch");
  yaw_pid_ = std::make_unique<control_toolbox::PidROS>(get_node(), "yaw");

  if (pitch_pid_->initPid() == false || yaw_pid_->initPid() == false) {
    fprintf(stderr, "PID params can't be set!");
    return CallbackReturn::FAILURE;
  }

  imu_sensor_ =
    std::make_unique<semantic_components::IMUSensor>(semantic_components::IMUSensor(sensor_name_));

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RMGimbalController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = imu_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

CallbackReturn RMGimbalController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = forward_command_controller::ForwardCommandController::on_activate(previous_state);

  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  return ret;
}

CallbackReturn RMGimbalController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);

  // stop all joints
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  imu_sensor_->release_interfaces();

  return ret;
}

controller_interface::return_type RMGimbalController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto joint_position_commands = *rt_command_ptr_.readFromRT();
  // no command received yet
  if (!joint_position_commands) {
    return controller_interface::return_type::OK;
  }

  RCLCPP_INFO(logger_, "update");

  // get imu orientation
  auto orientation = imu_sensor_->get_orientation();
  auto q = tf2::Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
  // transform to RPY
  double current_roll, current_pitch, current_yaw;
  tf2::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(current_roll, current_pitch, current_yaw);

  double command_pitch = joint_position_commands->data[0];
  double command_yaw = joint_position_commands->data[1];

  auto pitch_error = angles::shortest_angular_distance(current_pitch, command_pitch);
  auto yaw_error = angles::shortest_angular_distance(current_yaw, command_yaw);

  // TODO(chenjun): enforcing joint limits
  pitch_pid_->computeCommand(pitch_error, period);
  yaw_pid_->computeCommand(yaw_error, period);

  return controller_interface::return_type::OK;
}

}  // namespace rm_gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rm_gimbal_controller::RMGimbalController, controller_interface::ControllerInterface)
