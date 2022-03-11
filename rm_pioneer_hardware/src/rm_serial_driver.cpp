// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_serial_driver/rm_serial_driver.hpp"

#include <rclcpp/logging.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_pioneer_hardware
{
RMSerialDriver::RMSerialDriver(const std::unordered_map<std::string, std::string> & params)
: logger_(rclcpp::get_logger("SerialDriver")),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(logger_, "Start RMSerialDriver!");

  resolveParams(params);

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
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
    RCLCPP_ERROR(logger_, "Error sending data: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
}

ReceivePacket RMSerialDriver::readData()
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
        return packet;
      } else {
        RCLCPP_ERROR(logger_, "CRC error!");
      }
    } else {
      RCLCPP_WARN(logger_, "Invalid header: %02X", header[0]);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Error while receiving data: %s", ex.what());
    throw ex;
  }

  return ReceivePacket();
}

void RMSerialDriver::writeCommand(const double & pitch_command, const double & yaw_command)
{
  try {
    SendPacket packet;
    packet.is_request = false;
    packet.pitch_command = static_cast<int16_t>(pitch_command);
    packet.yaw_command = static_cast<int16_t>(yaw_command);
    crc16::appendCRC16CheckSum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Error while writing data: %s", ex.what());
    throw ex;
  }
}

void RMSerialDriver::resolveParams(const std::unordered_map<std::string, std::string> & params)
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  device_name_ = params.at("device_name");

  baud_rate = std::stoi(params.at("baud_rate"));

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

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

}  // namespace rm_pioneer_hardware
