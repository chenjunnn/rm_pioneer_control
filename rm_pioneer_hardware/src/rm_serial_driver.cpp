// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_pioneer_hardware/rm_serial_driver.hpp"

#include <rcl/visibility_control.h>

// ROS
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rm_pioneer_hardware/crc.hpp"
#include "rm_pioneer_hardware/packet.hpp"

namespace rm_pioneer_hardware
{
RMSerialDriver::RMSerialDriver(const std::unordered_map<std::string, std::string> & params)
: Node("SerialDriver"),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  resolveParams(params);

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
}

RMSerialDriver::~RMSerialDriver()
{
  serial_driver_->port()->close();

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::sendRequest()
{
  SendPacket packet;
  packet.is_request = true;
  crc16::appendCRC16CheckSum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

  auto data = toVector(packet);

  try {
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error sending data: %s - %s", device_name_.c_str(), ex.what());
    reopenPort();
  }
}

void RMSerialDriver::readData(std::array<double, 6> & imu_raw_data)
{
  try {
    std::vector<uint8_t> header(1);
    serial_driver_->port()->receive(header);

    if (header[0] == 0x5A) {
      std::vector<uint8_t> data(sizeof(ReceivePacket) - 1);
      serial_driver_->port()->receive(data);

      ReceivePacket packet = fromVector(data);
      bool crc_ok =
        crc16::verifyCRC16CheckSum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
      if (crc_ok) {
        imu_raw_data[0] = packet.linear_acceleration_x;
        imu_raw_data[1] = packet.linear_acceleration_y;
        imu_raw_data[2] = packet.linear_acceleration_z;
        imu_raw_data[3] = packet.angular_velocity_x;
        imu_raw_data[4] = packet.angular_velocity_y;
        imu_raw_data[5] = packet.angular_velocity_z;
      } else {
        RCLCPP_ERROR(get_logger(), "CRC error!");
      }
    } else {
      RCLCPP_WARN(get_logger(), "Invalid header: %02X", header[0]);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
    reopenPort();
  }
}

// void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
// {
//   SendPacket packet;
//   packet.target_found = msg->target_found;
//   packet.x = msg->position.x;
//   packet.y = msg->position.y;
//   packet.z = msg->position.z;
//   packet.vx = msg->velocity.x;
//   packet.vy = msg->velocity.y;
//   packet.vz = msg->velocity.z;
//   crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

//   std::vector<uint8_t> data = toVector(packet);

//   if (serial_driver_->port()->is_open()) {
//     serial_driver_->port()->send(data);
//   } else {
//     RCLCPP_WARN(get_logger(), "Serial port is not open, ignore sending data!");
//   }

//   std_msgs::msg::Float64 latency;
//   latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
//   latency_pub_->publish(latency);
// }

void RMSerialDriver::resolveParams(const std::unordered_map<std::string, std::string> & params)
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = params.at("device_name");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = std::stoi(params.at("baud_rate"));
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = params.at("flow_control");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = params.at("parity");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = params.at("stop_bits");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    rclcpp::sleep_for(std::chrono::seconds(1));
    reopenPort();
  }
}

}  // namespace rm_pioneer_hardware
