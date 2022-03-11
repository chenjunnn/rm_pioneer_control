// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_gimbal_controller/rm_gimbal_controller.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>

#include <algorithm>
#include <controller_interface/controller_interface.hpp>
#include <forward_command_controller/forward_command_controller.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

// STL
#include <memory>
#include <rclcpp/qos.hpp>
#include <string>
#include <vector>

namespace rm_gimbal_controller
{
RMGimbalController::RMGimbalController() : forward_command_controller::ForwardCommandController()
{
  interface_name_ = hardware_interface::HW_IF_EFFORT;
  joint_names_ = std::vector<std::string>{"pitch_joint", "yaw_joint"};
}

CallbackReturn RMGimbalController::on_init()
{
  auto ret = forward_command_controller::ForwardCommandController::on_init();
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    // Explicitly set the joints and interface parameter declared by the forward_command_controller
    get_node()->set_parameter(rclcpp::Parameter("joints", joint_names_));
    get_node()->set_parameter(
      rclcpp::Parameter("interface_name", hardware_interface::HW_IF_EFFORT));

    pitch_pid_ = std::make_unique<control_toolbox::PidROS>(node_, "pitch_joint");
    yaw_pid_ = std::make_unique<control_toolbox::PidROS>(node_, "yaw_joint");
    pitch_pid_->initPid();
    yaw_pid_->initPid();

    // Reset PID parameters event handler that was set previously by PidROS
    node_->remove_on_set_parameters_callback(pitch_pid_->getParametersCallbackHandle().get());
    node_->remove_on_set_parameters_callback(yaw_pid_->getParametersCallbackHandle().get());
    setParameterEventCallback();

    // Get imu sensor name and create IMU sensor
    sensor_name_ = auto_declare<std::string>("sensor_name", "");
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_name_));

    // Initialize the joint state publisher
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    rt_js_pub_ = std::make_shared<RealtimeJointStatePublisher>(joint_state_pub_);
    rt_js_pub_->msg_.name.emplace_back("pitch_joint");
    rt_js_pub_->msg_.name.emplace_back("yaw_joint");

    // Initialize joint limits
    urdf::Model urdf_model;
    urdf_model.initString(node_->get_parameter("robot_description").as_string());
    pitch_upper_limit = urdf_model.joints_["pitch_joint"]->limits->upper;
    pitch_lower_limit = urdf_model.joints_["pitch_joint"]->limits->lower;
    pitch_effort_limit = urdf_model.joints_["pitch_joint"]->limits->effort;
    yaw_effort_limit = urdf_model.joints_["yaw_joint"]->limits->effort;
    RCLCPP_INFO(node_->get_logger(), "Pitch joint upper limit: %f", pitch_upper_limit);
    RCLCPP_INFO(node_->get_logger(), "Pitch joint lower limit: %f", pitch_lower_limit);
    RCLCPP_INFO(node_->get_logger(), "Pitch joint effort limit: %f", pitch_effort_limit);
    RCLCPP_INFO(node_->get_logger(), "Yaw joint effort limit: %f", yaw_effort_limit);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

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
  // get imu orientation
  auto orientation = imu_sensor_->get_orientation();
  auto q = tf2::Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
  // transform to RPY
  double current_roll, current_pitch, current_yaw;
  tf2::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(current_roll, current_pitch, current_yaw);

  // publish joint states
  if (rt_js_pub_) {
    if (rt_js_pub_->trylock()) {
      rt_js_pub_->msg_.header.stamp = time;
      rt_js_pub_->msg_.position.clear();
      rt_js_pub_->msg_.position.emplace_back(current_pitch);
      rt_js_pub_->msg_.position.emplace_back(current_yaw);
      rt_js_pub_->unlockAndPublish();
    }
  }

  auto joint_position_commands = *rt_command_ptr_.readFromRT();
  // no command received yet
  if (!joint_position_commands) {
    return controller_interface::return_type::OK;
  }

  double command_pitch = joint_position_commands->data[0];
  double command_yaw = joint_position_commands->data[1];

  // limit pitch position command
  command_pitch = std::clamp(command_pitch, pitch_lower_limit, pitch_upper_limit);

  auto pitch_error = angles::shortest_angular_distance(current_pitch, command_pitch);
  auto yaw_error = angles::shortest_angular_distance(current_yaw, command_yaw);

  double pitch_cmd = pitch_pid_->computeCommand(pitch_error, period);
  double yaw_cmd = yaw_pid_->computeCommand(yaw_error, period);

  // enforce output limits
  pitch_cmd = std::clamp(pitch_cmd, -pitch_effort_limit, pitch_effort_limit);
  yaw_cmd = std::clamp(yaw_cmd, -yaw_effort_limit, yaw_effort_limit);

  // set command
  command_interfaces_[0].set_value(pitch_cmd);
  command_interfaces_[1].set_value(yaw_cmd);

  return controller_interface::return_type::OK;
}

void RMGimbalController::setParameterEventCallback()
{
  auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;

    auto pitch_gains = pitch_pid_->getGains();
    auto yaw_gains = yaw_pid_->getGains();

    auto set_pid_parameters =
      [&](const std::string & joint_name, control_toolbox::Pid::Gains & gains) -> bool {
      result.successful = true;
      for (auto & parameter : parameters) {
        const std::string param_name = parameter.get_name();
        try {
          if (param_name == joint_name + ".p") {
            gains.p_gain_ = parameter.get_value<double>();
          } else if (param_name == joint_name + ".i") {
            gains.i_gain_ = parameter.get_value<double>();
          } else if (param_name == joint_name + ".d") {
            gains.d_gain_ = parameter.get_value<double>();
          } else if (param_name == joint_name + ".i_clamp_max") {
            gains.i_max_ = parameter.get_value<double>();
          } else if (param_name == joint_name + ".i_clamp_min") {
            gains.i_min_ = parameter.get_value<double>();
          } else if (param_name == joint_name + ".antiwindup") {
            gains.antiwindup_ = parameter.get_value<bool>();
          } else {
            result.successful = false;
            result.reason = "Invalid parameter " + param_name;
          }
        } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
          RCLCPP_INFO_STREAM(node_->get_logger(), "Please use the right type: " << e.what());
        }
      }
      return result.successful;
    };

    if (set_pid_parameters("pitch_joint", pitch_gains)) {
      pitch_pid_->setGains(pitch_gains);
    } else if (set_pid_parameters("yaw_joint", yaw_gains)) {
      yaw_pid_->setGains(yaw_gains);
    }
    return result;
  };

  set_param_handle_ = node_->add_on_set_parameters_callback(on_parameter_event_callback);
}

}  // namespace rm_gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rm_gimbal_controller::RMGimbalController, controller_interface::ControllerInterface)
