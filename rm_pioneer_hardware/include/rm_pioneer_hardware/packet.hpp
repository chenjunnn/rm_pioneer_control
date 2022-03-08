// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_PIONEER_HARDWARE__PACKET_HPP_
#define RM_PIONEER_HARDWARE__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_pioneer_hardware
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  float pitch;
  float yaw;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool target_found;
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
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

#endif  // RM_PIONEER_HARDWARE__PACKET_HPP_
