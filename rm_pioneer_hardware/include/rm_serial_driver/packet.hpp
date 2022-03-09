// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_pioneer_hardware
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  float angular_velocity_x = 0.f;
  float angular_velocity_y = 0.f;
  float angular_velocity_z = 0.f;
  float linear_acceleration_x = 0.f;
  float linear_acceleration_y = 0.f;
  float linear_acceleration_z = 0.f;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool is_request = false;
  bool target_found = false;
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float vx = 0.f;
  float vy = 0.f;
  float vz = 0.f;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet) + 1);
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_pioneer_hardware

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
