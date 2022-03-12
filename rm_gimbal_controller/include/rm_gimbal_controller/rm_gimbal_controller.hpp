// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_GIMBAL_CONTROLLER__RM_GIMBAL_CONTROLLER_HPP_
#define RM_GIMBAL_CONTROLLER__RM_GIMBAL_CONTROLLER_HPP_

#include <urdf/model.h>

#include <control_msgs/msg/pid_state.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <forward_command_controller/forward_command_controller.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <semantic_components/imu_sensor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>

// STL
#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_gimbal_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using RealtimePIDStatePublisher = realtime_tools::RealtimePublisher<control_msgs::msg::PidState>;
using RealtimeJointStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;
using CmdType = sensor_msgs::msg::Joy;
using TargetMsg = auto_aim_interfaces::msg::Target;

class RMGimbalController : public forward_command_controller::ForwardCommandController
{
public:
  RMGimbalController();

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void setParameterEventCallback();

  std::unique_ptr<control_toolbox::PidROS> pitch_pid_;
  std::unique_ptr<control_toolbox::PidROS> yaw_pid_;

  std::string sensor_name_;
  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_pub_;
  std::shared_ptr<RealtimeJointStatePublisher> rt_js_pub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TargetMsg>> rt_target_ptr_;
  rclcpp::Subscription<TargetMsg>::SharedPtr auto_aim_target_subscriber_;

  double pitch_upper_limit_;
  double pitch_lower_limit_;
  double pitch_velocity_limit_;
  double pitch_effort_limit_;

  double yaw_velocity_limit_;
  double yaw_effort_limit_;

  double pitch_position_command_;
  double yaw_position_command_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_handle_;
};
}  // namespace rm_gimbal_controller

#endif  // RM_GIMBAL_CONTROLLER__RM_GIMBAL_CONTROLLER_HPP_
