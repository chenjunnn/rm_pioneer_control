# Copyright (c) 2022 ChenJun
# Licensed under the MIT License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

TOP_BOUNDARY = 2.0
BOTTOM_BOUNDARY = 1.4
LEFT_BOUNDARY = 1.3
RIGHT_BOUNDARY = 3.0

PITCH_SPEED = 1.0
YAW_SPEED = 1.0


class RMGuardPatroller(Node):

    def __init__(self):
        super().__init__('rm_guard_patroller')

        self.subscription = self.create_subscription(
            Vector3,
            '/motor_position',
            self.guard_patroller_cmd_callback,
            10)

        self.publisher = self.create_publisher(
            Vector3,
            '/rm_gimbal_controller/commands',
            10)

        self.speed = Vector3()
        self.speed.y = PITCH_SPEED
        self.speed.z = YAW_SPEED

    def guard_patroller_cmd_callback(self, msg):
        # IMU角度方向和电机角度是相反的，所以速度值需要取反
        # Pitch
        if msg.y >= TOP_BOUNDARY:
            self.speed.y = PITCH_SPEED
        elif msg.y <= BOTTOM_BOUNDARY:
            self.speed.y = -PITCH_SPEED

        # Yaw
        if msg.z <= LEFT_BOUNDARY:
            self.speed.z = -YAW_SPEED
        elif msg.z >= RIGHT_BOUNDARY:
            self.speed.z = YAW_SPEED

        self.publisher.publish(self.speed)


def main(args=None):
    rclpy.init(args=args)

    rm_guard_patroller = RMGuardPatroller()

    rclpy.spin(rm_guard_patroller)

    rm_guard_patroller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
