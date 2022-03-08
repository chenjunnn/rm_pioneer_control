// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_PIONEER_HARDWARE__RM_SERIAL_DRIVER_HPP_
#define RM_PIONEER_HARDWARE__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/logger.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace rm_pioneer_hardware
{
class RMSerialDriver
{
public:
  explicit RMSerialDriver(const std::unordered_map<std::string, std::string> & params);

  ~RMSerialDriver();

private:
  void resolveParams(const std::unordered_map<std::string, std::string> & params);

  void receiveData();

  // void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void reopenPort();

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Logger logger_;

  std::thread receive_thread_;
};
}  // namespace rm_pioneer_hardware

#endif  // RM_PIONEER_HARDWARE__RM_SERIAL_DRIVER_HPP_
