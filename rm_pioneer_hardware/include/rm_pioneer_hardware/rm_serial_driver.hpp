// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_PIONEER_HARDWARE__RM_SERIAL_DRIVER_HPP_
#define RM_PIONEER_HARDWARE__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

namespace rm_pioneer_hardware
{
class RMSerialDriver : rclcpp::Node
{
public:
  explicit RMSerialDriver(const std::unordered_map<std::string, std::string> & params);

  ~RMSerialDriver();

  void sendRequest();
  void readData(std::array<double, 6> & imu_raw_data);

  void writeCommand();

  inline rclcpp::Time getTime() const { return this->now(); }

private:
  void resolveParams(const std::unordered_map<std::string, std::string> & params);

  void reopenPort();

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};
}  // namespace rm_pioneer_hardware

#endif  // RM_PIONEER_HARDWARE__RM_SERIAL_DRIVER_HPP_
