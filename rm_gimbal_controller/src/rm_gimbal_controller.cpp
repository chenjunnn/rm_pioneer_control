// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_gimbal_controller/rm_gimbal_controller.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>

#include <controller_interface/controller_interface.hpp>
#include <forward_command_controller/forward_command_controller.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rm_gimbal_controller
{
RMGimbalController::RMGimbalController() : forward_command_controller::ForwardCommandController()
{
  joint_names_ = {"pitch_joint", "yaw_joint", "shooter"};
  interface_name_ = hardware_interface::HW_IF_EFFORT;

  pitch_position_command_ = 0;
  yaw_position_command_ = 0;
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

    // Initialize the marker publisher
    marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    rt_marker_pub_ = std::make_shared<RealtimeMarkerPublisher>(marker_pub_);
    rt_marker_pub_->msg_.header.frame_id = "shooter_link";
    rt_marker_pub_->msg_.ns = "gimbal_controller";
    rt_marker_pub_->msg_.id = 0;
    rt_marker_pub_->msg_.type = visualization_msgs::msg::Marker::POINTS;
    rt_marker_pub_->msg_.action = visualization_msgs::msg::Marker::ADD;
    rt_marker_pub_->msg_.scale.x = 0.1;
    rt_marker_pub_->msg_.scale.y = 0.1;
    rt_marker_pub_->msg_.scale.z = 0.1;
    rt_marker_pub_->msg_.color.a = 1.0;
    rt_marker_pub_->msg_.color.r = 0.0;
    rt_marker_pub_->msg_.color.g = 1.0;
    rt_marker_pub_->msg_.color.b = 1.0;
    rt_marker_pub_->msg_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // Initialize joint limits
    urdf::Model urdf_model;
    urdf_model.initString(node_->get_parameter("robot_description").as_string());
    // Pitch limits
    auto pitch_joint_limits = urdf_model.getJoint("pitch_joint")->limits;
    pitch_upper_limit_ = pitch_joint_limits->upper;
    pitch_lower_limit_ = pitch_joint_limits->lower;
    pitch_velocity_limit_ = pitch_joint_limits->velocity;
    pitch_effort_limit_ = pitch_joint_limits->effort;
    RCLCPP_INFO(node_->get_logger(), "Pitch joint upper limit: %f", pitch_upper_limit_);
    RCLCPP_INFO(node_->get_logger(), "Pitch joint lower limit: %f", pitch_lower_limit_);
    RCLCPP_INFO(node_->get_logger(), "Pitch joint velocity limit: %f", pitch_velocity_limit_);
    RCLCPP_INFO(node_->get_logger(), "Pitch joint effort limit: %f", pitch_effort_limit_);
    // Yaw limits
    auto yaw_joint_limits = urdf_model.getJoint("yaw_joint")->limits;
    yaw_velocity_limit_ = yaw_joint_limits->velocity;
    yaw_effort_limit_ = yaw_joint_limits->effort;
    RCLCPP_INFO(node_->get_logger(), "Yaw joint effort limit: %f", yaw_effort_limit_);
    RCLCPP_INFO(node_->get_logger(), "Yaw joint velocity limit: %f", yaw_velocity_limit_);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RMGimbalController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  auto_aim_target_subscriber_ = get_node()->create_subscription<TargetMsg>(
    "~/target", rclcpp::SensorDataQoS(),
    [this](const TargetMsg::SharedPtr msg) { rt_target_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
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

  // reset buffer if a target msg came through callback when controller was inactive
  rt_target_ptr_.reset();

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

  // reset target buffer
  rt_target_ptr_.reset();

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

  current_pitch = current_roll;

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

  // auto joint_commands = rt_command_ptr_.readFromRT();
  auto target_msg = rt_target_ptr_.readFromRT();

  // no command received yet
  // if (!joint_commands || !(*joint_commands)) {
  //   return controller_interface::return_type::OK;
  // }

  pitch_velocity_command_ = 0.0;
  yaw_velocity_command_ = 0.0;

  // Target found
  bool target_found = false;
  if (target_msg && *target_msg && target_msg->get()->target_found) {
    target_found = true;

    double latency = (time - target_msg->get()->header.stamp).seconds();
    double origin_x = target_msg->get()->position.x;
    double origin_y = target_msg->get()->position.y;
    double origin_z = target_msg->get()->position.z;

    double velocity_x = target_msg->get()->velocity.x;
    double velocity_y = target_msg->get()->velocity.y;
    double velocity_z = target_msg->get()->velocity.z;

    double predict_x = origin_x + velocity_x * (latency + 0.2);
    double predict_y = origin_y + velocity_y * (latency + 0.2);
    double predict_z = origin_z + velocity_z * (latency + 0.2);

    // Publish marker
    if (rt_marker_pub_) {
      if (rt_marker_pub_->trylock()) {
        rt_marker_pub_->msg_.header.stamp = time;
        rt_marker_pub_->msg_.points.clear();
        geometry_msgs::msg::Point p;
        p.x = predict_x;
        p.y = predict_y;
        p.z = predict_z;
        rt_marker_pub_->msg_.points.emplace_back(p);
        rt_marker_pub_->unlockAndPublish();
      }
    }

    double distance = std::sqrt(std::pow(predict_x, 2) + std::pow(predict_y, 2));
    pitch_position_command_ = -std::atan2(predict_z, distance);

    yaw_position_command_ = std::atan2(predict_y, predict_x);
  }

  // limit pitch position command
  pitch_position_command_ =
    std::clamp(pitch_position_command_, pitch_lower_limit_, pitch_upper_limit_);

  double pitch_position_error =
    angles::shortest_angular_distance(current_pitch, pitch_position_command_);
  double pitch_velocity_error = pitch_velocity_command_ - imu_sensor_->get_angular_velocity()[0];
  double pitch_output =
    pitch_pid_->computeCommand(pitch_position_error, pitch_velocity_error, period);

  double yaw_position_error = angles::shortest_angular_distance(current_yaw, yaw_position_command_);
  double yaw_velocity_error = yaw_velocity_command_ - imu_sensor_->get_angular_velocity()[2];
  double yaw_output = yaw_pid_->computeCommand(yaw_position_error, yaw_velocity_error, period);

  // enforce output limits
  pitch_output = std::clamp(pitch_output, -pitch_effort_limit_, pitch_effort_limit_);
  yaw_output = std::clamp(yaw_output, -yaw_effort_limit_, yaw_effort_limit_);

  // set output
  command_interfaces_[0].set_value(pitch_output);
  command_interfaces_[1].set_value(yaw_output);
  command_interfaces_[2].set_value(target_found);

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
