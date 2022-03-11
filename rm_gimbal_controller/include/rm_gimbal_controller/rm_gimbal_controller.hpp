// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_GIMBAL_CONTROLLER__RM_GIMBAL_CONTROLLER_HPP_
#define RM_GIMBAL_CONTROLLER__RM_GIMBAL_CONTROLLER_HPP_

// ROS
#include <control_toolbox/pid_ros.hpp>
#include <forward_command_controller/forward_command_controller.hpp>
#include <semantic_components/imu_sensor.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// STL
#include <memory>
#include <string>

namespace rm_gimbal_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RMGimbalController : public forward_command_controller::ForwardCommandController
{
public:
  RMGimbalController();

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<control_toolbox::PidROS> pitch_pid_;
  std::unique_ptr<control_toolbox::PidROS> yaw_pid_;

  // IMU sensor
  std::string sensor_name_;
  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

  rclcpp::Logger logger_;
};
}  // namespace rm_gimbal_controller

#endif  // RM_GIMBAL_CONTROLLER__RM_GIMBAL_CONTROLLER_HPP_
