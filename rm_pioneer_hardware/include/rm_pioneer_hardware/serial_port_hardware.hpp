// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_PIONEER_HARDWARE__SERIAL_PORT_HARDWARE_HPP_
#define RM_PIONEER_HARDWARE__SERIAL_PORT_HARDWARE_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rm_pioneer_hardware/rm_serial_driver.hpp"

namespace rm_pioneer_hardware
{
class SerialPortHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

private:
  std::vector<double> hw_joint_commands_;
  std::vector<double> hw_joint_states_;
  std::vector<double> hw_sensor_states_;

  std::unique_ptr<RMSerialDriver> serial_driver_;
};
}  // namespace rm_pioneer_hardware

#endif  // RM_PIONEER_HARDWARE__SERIAL_PORT_HARDWARE_HPP_
