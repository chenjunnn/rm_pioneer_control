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

namespace rm_pioneer_hardware
{
class RMSerialDriver
{
public:
  explicit RMSerialDriver(const std::unordered_map<std::string, std::string> & params);

  ~RMSerialDriver();

  void sendRequest();
  void readData(std::array<double, 6> & imu_raw_data);

  void writeCommand();

private:
  void resolveParams(const std::unordered_map<std::string, std::string> & params);

  rclcpp::Logger logger_;

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
};
}  // namespace rm_pioneer_hardware

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
