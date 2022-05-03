// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/logger.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rm_serial_driver/packet.hpp"

namespace rm_pioneer_hardware
{
class RMSerialDriver
{
public:
  explicit RMSerialDriver(const std::unordered_map<std::string, std::string> & params);

  ~RMSerialDriver();

  void sendRequest();
  ReceivePacket readData();

  void writeCommand(
    const double & pitch_command, const double & yaw_command, const bool & shoot_cmd);

private:
  void resolveParams(const std::unordered_map<std::string, std::string> & params);

  void reopenPort();

  rclcpp::Logger logger_;

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};
}  // namespace rm_pioneer_hardware

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
